using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

public class IntegratedHapticNode : HapticNode
{
    [Tooltip("Treat the articulation hierarchy as one equivalent rigid body.")]
    public bool useChainedMass = false;

    // Tracked node kinematics
    private Vector3 trackedPos = Vector3.zero;
    private Vector3 prevTrackedPos = Vector3.zero;
    private Quaternion trackedRot = Quaternion.identity;
    private Quaternion prevTrackedRot = Quaternion.identity;
    private Vector3 trackedVel = Vector3.zero;
    private Vector3 trackedAccel = Vector3.zero;
    private Vector3 prevTrackedVel = Vector3.zero;
    private Vector3 trackedAngVel = Vector3.zero;
    private Vector3 prevTrackedAngVel = Vector3.zero;

    // ArticulationBody of the physics object
    private ArticulationBody physicsAb;

    // Mass of articulation body
    private float physicsMass;

    // Intertial matrix of articulation body (world space)
    private Matrix4x4 physicsInertia;

    //Physics node kinematics
    private Vector3 physicsVel = Vector3.zero;
    private Vector3 prevPhysicsVel = Vector3.zero;
    private Vector3 physicsAccel = Vector3.zero;
    private Vector3 prevPhysicsForce = Vector3.zero;
    private float nextDiagnosticsTime;
    private int fixedStep;
    private Vector3 previousRelativePosition = Vector3.zero;
    private bool hasPreviousRelativePosition;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    new protected void Start()
    {
        // Initializes haptic node
        base.Start();

        // Gets physics object rigidbody
        physicsAb = gameObject.GetComponent<ArticulationBody>();
        UpdateMassProperties();
        UpdateKinematics();
    }

    private void SetInitialMassProperties()
    {
        physicsMass = physicsAb.mass;
        physicsInertia = Utils.WorldInertiaMatrix(physicsAb);
    }

    public void SetChainedMassProperties()
    {
        Utils.GetChainedMassProperties(physicsAb, out float mass, out _, out Matrix4x4 inertia);
        physicsMass = mass;
        physicsInertia = inertia;
    }

    private void UpdateMassProperties()
    {
        if (useChainedMass)
            SetChainedMassProperties();
        else
            SetInitialMassProperties();

    }

    // Fixed update is called once per physics frame
    public void UpdatePhysics()
    {
        fixedStep++;
        UpdateMassProperties();

        // Update kinematics of the tracked object
        UpdateKinematics();
        // Calculate the force and torque on the physics object
        Vector3 newPhysicsForce = ComputeForceOnPhysics();
        Vector3 newPhysicsTorque = ComputeTorqueOnPhysics();

        // Trims by max force
        if (newPhysicsForce.magnitude > maxPhysicsForce)
        {
            newPhysicsForce = newPhysicsForce.normalized * maxPhysicsForce;
        }
        physicsAb.AddForce(newPhysicsForce);

        // Trims by max torque
        if (newPhysicsTorque.magnitude > maxPhysicsTorque)
        {
            newPhysicsTorque = newPhysicsTorque.normalized * maxPhysicsTorque;
        }
        physicsAb.AddTorque(newPhysicsTorque);

        // Updates haptic node state
        Vector3 inertiaForce = -physicsMass * physicsAccel;
        UpdateState(-prevPhysicsForce, inertiaForce);

        LogDiagnostics(newPhysicsForce, newPhysicsTorque, inertiaForce);

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
        prevTrackedVel = trackedVel;
        trackedVel = (trackedPos - prevTrackedPos) / Time.fixedDeltaTime;
        trackedAccel = (trackedVel - prevTrackedVel) / Time.fixedDeltaTime;

        // Tracked rotation kinematics
        prevTrackedRot = trackedRot;
        trackedRot = trackedTransform.rotation;
        prevTrackedAngVel = trackedAngVel;
        trackedAngVel = Utils.AngularVelocityFromQuaternions(prevTrackedRot, trackedRot, Time.fixedDeltaTime);

        // Physics acceleration kinematics
        prevPhysicsVel = physicsVel;
        physicsVel = physicsAb.linearVelocity;
        physicsAccel = (physicsVel - prevPhysicsVel) / Time.fixedDeltaTime;
    }

    private Vector3 ComputeForceOnPhysics()
    {
        if (posStiffness == 0)
        {
            return Vector3.zero;
        }

        Vector3 relPos = physicsAb.transform.position - trackedPos;

        Vector3 relVel = physicsAb.linearVelocity - trackedVel;

        Vector3 accel =
            -(Utils.GetMaxStiffness() * posStiffness * relPos
            + Utils.GetMaxDamping() * posDamping * relVel);
        Vector3 force = physicsMass * accel;
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
        // Get orientations and angular velocities
        Quaternion trackedRotCopy = trackedRot;
        Quaternion physicsRot = gameObject.transform.rotation;

        Vector3 physicsAngVel = physicsAb.angularVelocity;

        // Calculate the relative rotation from tracked to physics in global coordinates
        Vector3 relRot = Utils.AngularVelocityFromQuaternions(
            trackedRotCopy,
            physicsRot,
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

    public Vector3 GetPhysicsAccel()
    {
        return physicsAccel;
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
            $"physics pos={physicsAb.transform.position:F4}, vel={physicsVel:F4}, " +
            $"accel={physicsAccel}, mass={physicsMass:F6}\n" +
            $"force physics={forceOnPhysics}, |force|={forceOnPhysics.magnitude:F6}, " +
            $"inertia force={inertiaForce}, |inertia|={inertiaForce.magnitude:F6}, " +
            $"torque={torqueOnPhysics}, |torque|={torqueOnPhysics.magnitude:F6}");
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);    }
}
