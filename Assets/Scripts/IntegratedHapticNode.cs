using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

public class IntegratedHapticNode : HapticNode
{
    // Tracked node kinematics
    private Vector3 trackedPos = Vector3.zero;
    private Vector3 prevTrackedPos = Vector3.zero;
    private Vector3 trackedComPos = Vector3.zero;
    private Quaternion trackedRot = Quaternion.identity;
    private Quaternion prevTrackedRot = Quaternion.identity;
    private Vector3 trackedVel = Vector3.zero;
    private Vector3 trackedAccel = Vector3.zero;
    private Vector3 prevTrackedVel = Vector3.zero;
    private Vector3 trackedAngVel = Vector3.zero;
    private Vector3 prevTrackedAngVel = Vector3.zero;

    // RigidBody of the physics object
    private Rigidbody physicsRb;
    private ArticulationBody articulationBody;
    private bool physicsBodyErrorLogged;

    //Physics node kinematics
    private Vector3 physicsVel = Vector3.zero;
    private Vector3 prevPhysicsVel = Vector3.zero;
    private Vector3 physicsAccel = Vector3.zero;
    private Vector3 physicsAngVel = Vector3.zero;
    private Vector3 prevPhysicsAngVel = Vector3.zero;
    private Vector3 physicsAngAccel = Vector3.zero;
    private Vector3 prevPhysicsForce = Vector3.zero;
    private Matrix4x4 physicsInertia = Matrix4x4.zero;
    private float nextDiagnosticsTime;
    private int fixedStep;
    private Vector3 previousRelativePosition = Vector3.zero;
    private bool hasPreviousRelativePosition;

    public bool LastPhysicsForceReachedLimit { get; private set; }
    public float LastPhysicsForceMagnitude { get; private set; }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    new protected void Start()
    {
        // Initializes haptic node
        base.Start();

        // Gets physics object rigidbody
        TryGetPhysicsBody();
        UpdateKinematics();
    }

    private bool TryGetPhysicsBody()
    {
        if (physicsRb != null || articulationBody != null)
            return true;

        if (gameObject.TryGetComponent<Rigidbody>(out physicsRb))
            return true;

        if (gameObject.TryGetComponent<ArticulationBody>(out articulationBody))
            return true;

        if (!physicsBodyErrorLogged)
        {
            Debug.LogError(
                $"[{name}] IntegratedHapticNode requires a Rigidbody or ArticulationBody.");
            physicsBodyErrorLogged = true;
        }

        return false;
    }

    // Fixed update is called once per physics frame
    public void UpdatePhysics()
    {
        if (!TryGetPhysicsBody())
            return;

        fixedStep++;

        // Update kinematics of the tracked object
        UpdateKinematics();
        // Calculate the force and torque on the physics object
        Vector3 newPhysicsForce = ComputeForceOnPhysics();
        Vector3 newPhysicsTorque = ComputeTorqueOnPhysics();

        // Trims by max force
        LastPhysicsForceMagnitude = newPhysicsForce.magnitude;
        LastPhysicsForceReachedLimit =
            maxPhysicsForce > 0f &&
            newPhysicsForce.magnitude >= maxPhysicsForce;
        if (newPhysicsForce.magnitude > maxPhysicsForce)
        {
            newPhysicsForce = newPhysicsForce.normalized * maxPhysicsForce;
        }

        // Trims by max torque
        if (newPhysicsTorque.magnitude > maxPhysicsTorque)
        {
            newPhysicsTorque = newPhysicsTorque.normalized * maxPhysicsTorque;
        }

        if (physicsRb != null)
        {
            physicsRb.AddTorque(newPhysicsTorque);
            physicsRb.AddForce(newPhysicsForce);
        }
        else
        {
            articulationBody.AddTorque(newPhysicsTorque);
            articulationBody.AddForce(newPhysicsForce);
        }

        // Updates haptic node state
        Vector3 inertiaForce = -GetPhysicsMass() * physicsAccel;
        UpdateState(-prevPhysicsForce, inertiaForce);

        //LogDiagnostics(newPhysicsForce, newPhysicsTorque, inertiaForce);

        prevPhysicsForce = newPhysicsForce;
    }

    // Update is called once per frame
    void Update()
    {
    }

    private void UpdateKinematics()
    {
        // Tracked position kinematics
        prevTrackedPos = trackedPos;
        trackedPos = trackedTransform.position;

        // Tracked rotation kinematics
        prevTrackedRot = trackedRot;
        trackedRot = trackedTransform.rotation;
        prevTrackedAngVel = trackedAngVel;
        trackedAngVel = Utils.AngularVelocityFromQuaternions(prevTrackedRot, trackedRot, Time.fixedDeltaTime);

        Vector3 localCenterOfMass = GetLocalCenterOfMass();
        trackedComPos = trackedPos + trackedRot * localCenterOfMass;
        Vector3 trackedOriginVel = (trackedPos - prevTrackedPos) / Time.fixedDeltaTime;
        trackedVel = trackedOriginVel +
            Vector3.Cross(trackedAngVel, trackedRot * localCenterOfMass);
        trackedAccel = (trackedVel - prevTrackedVel) / Time.fixedDeltaTime;
        prevTrackedVel = trackedVel;

        // Physics position kinematics
        prevPhysicsVel = physicsVel;
        physicsVel = GetPhysicsLinearVelocity();
        physicsAccel = (physicsVel - prevPhysicsVel) / Time.fixedDeltaTime;

        // Physics rotation kinematics
        prevPhysicsAngVel = physicsAngVel;
        physicsAngVel = GetPhysicsAngularVelocity();
        physicsAngAccel = (physicsAngVel - prevPhysicsAngVel) / Time.fixedDeltaTime;

        physicsInertia = physicsRb != null
            ? Utils.WorldInertiaMatrix(physicsRb)
            : Utils.WorldInertiaMatrix(articulationBody);
    }

    private float GetPhysicsMass()
    {
        return physicsRb != null ? physicsRb.mass : articulationBody.mass;
    }

    private Vector3 GetLocalCenterOfMass()
    {
        Vector3 worldCenterOfMass = physicsRb != null
            ? physicsRb.worldCenterOfMass
            : articulationBody.worldCenterOfMass;
        return transform.InverseTransformPoint(worldCenterOfMass);
    }

    private Vector3 GetPhysicsLinearVelocity()
    {
        return physicsRb != null
            ? physicsRb.linearVelocity
            : articulationBody.linearVelocity;
    }

    private Vector3 GetPhysicsAngularVelocity()
    {
        return physicsRb != null
            ? physicsRb.angularVelocity
            : articulationBody.angularVelocity;
    }

    private Vector3 ComputeForceOnPhysics()
    {
        if (posStiffness == 0)
        {
            return Vector3.zero;
        }

        Vector3 physicsComPos = physicsRb != null
            ? physicsRb.worldCenterOfMass
            : articulationBody.worldCenterOfMass;
        Vector3 relPos = physicsComPos - trackedComPos;

        Vector3 relVel = physicsVel - trackedVel;

        Vector3 accel =
            -(Utils.GetMaxStiffness() * posStiffness * relPos
            + Utils.GetMaxDamping() * posDamping * relVel);
        Vector3 force = GetPhysicsMass() * accel;
        //Print relative position and calculated force
        //Debug.Log($"Relative position: {relPos}, force: {force}");

        return force;
    }

    private Vector3 ComputeTorqueOnPhysics()
    {
        if (rotStiffness == 0)
        {
            return Vector3.zero;
        }
        
        // Calculate the relative rotation from tracked to physics in global coordinates
        Vector3 relRot = Utils.AngularVelocityFromQuaternions(
            trackedRot,
            transform.rotation,
            1);

        Vector3 relAngVel = physicsAngVel - trackedAngVel;

        Vector3 angAccel =
            -(Utils.GetMaxStiffness() * rotStiffness * relRot
            + Utils.GetMaxDamping() * rotDamping * relAngVel);
        // Apply the complete world-space inertia tensor. This preserves coupling
        // between axes when the aggregate tensor has products of inertia.
        Vector3 torque = Utils.MultiplyInertia(physicsInertia, angAccel);

        // Print relative rotation and calculated torque
        //Debug.Log($"Relative rotation axis: {relRot.normalized}, angle: {relRot.magnitude} rad, torque: {torque}");

        return torque;
    }

    private void LogDiagnostics(
        Vector3 forceOnPhysics,
        Vector3 torqueOnPhysics,
        Vector3 inertiaForce)
    {
        float timeStep = Time.fixedDeltaTime;

        Debug.Log(
            $"[{name}] Step={fixedStep}, t={Time.time:F4}, dt={timeStep:F6}\n" +
            $"tracked pos={trackedPos:F4}, prev={prevTrackedPos:F4}, " +
            $"vel={trackedVel}, prevVel={prevTrackedVel}\n" +
            $"physics pos={physicsRb.transform.position:F4}, vel={physicsVel:F4}, " +
            $"accel={physicsAccel}, mass={physicsRb.mass:F6}\n" +
            $"force physics={forceOnPhysics}, |force|={forceOnPhysics.magnitude:F6}, " +
            $"inertia force={inertiaForce}, |inertia|={inertiaForce.magnitude:F6}, " +
            $"torque={torqueOnPhysics}, |torque|={torqueOnPhysics.magnitude:F6}");
    }
}
