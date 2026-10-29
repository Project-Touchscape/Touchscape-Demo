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
    protected void FixedUpdate()
    {
        UpdateMassProperties();

        // Update kinematics of the tracked object
        UpdateKinematics();
        // Calculate the force and torque on the physics object
        Vector3 newForceOnPhysics = ComputeForceOnPhysics();
        Vector3 newTorqueOnPhysics = ComputeTorqueOnPhysics();

        // Gets inertia force to apply if applicable
        Vector3 inertiaForce = -physicsMass * physicsAccel;
        // Gets accurate force to apply on tracked object
        Vector3 trackedForce = GetForceOnTracked();
        UpdateWrench(trackedForce, newTorqueOnPhysics, inertiaForce);

        // Trims by max force
        /*if (newForceOnPhysics.magnitude > maxForce)
        {
            newForceOnPhysics = newForceOnPhysics.normalized * maxForce;
        }*/
        physicsAb.AddForce(newForceOnPhysics);

        // Trims by max torque
        /*if (newTorqueOnPhysics.magnitude > maxTorque)
        {
            newTorqueOnPhysics = newTorqueOnPhysics.normalized * maxTorque;
        }*/
        physicsAb.AddTorque(newTorqueOnPhysics);

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

        Debug.Log($"Tracked position: {trackedPos}, tracked velocity: {trackedVel}");
        Debug.Log($"Previous tracked position: {prevTrackedPos}, previous tracked velocity: {prevTrackedVel}");
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
        return ComputeForceOnPhysics(trackedTransform.position, trackedVel);
    }

    private Vector3 ComputeForceOnPhysics(Vector3 trackedPosition, Vector3 trackedVelocity)
    {
        if (posStiffness == 0 && posDamping == 0)
        {
            return Vector3.zero;
        }

        Vector3 relVel = physicsAb.linearVelocity - trackedVelocity;

        Vector3 relPos = physicsAb.transform.position - trackedPosition;

        // Applies coefficients
        Vector3 accel = -(maxStiffness * posStiffness * relPos + maxDamping * posDamping * relVel);
        Vector3 force = physicsMass * accel;
        //Print relative position and calculated force
        //Debug.Log($"Relative position: {relPos}, force: {force}");

        return force;
    }

    private Vector3 ComputeTorqueOnPhysics()
    {
        if (rotStiffness == 0 && rotDamping == 0)
        {
            return Vector3.zero;
        }
        // Get orientations and angular velocities
        Quaternion trackedRotCopy = trackedTransform.rotation;
        Quaternion physicsRot = gameObject.transform.rotation;

        Vector3 physicsAngVel = physicsAb.angularVelocity - trackedAngVel;

        // Calculate the relative rotation from tracked to physics in global coordinates
        Vector3 relRot = Utils.AngularVelocityFromQuaternions(trackedRotCopy, physicsRot, 1);

        // Applies coefficients to get necessary angular acceleration
        Vector3 angAccel = -(maxStiffness * rotStiffness * relRot + maxDamping * rotDamping * physicsAngVel);
        // Apply the complete world-space inertia tensor. This preserves coupling
        // between axes when the aggregate tensor has products of inertia.
        Vector3 torque = Utils.MultiplyInertia(physicsInertia, angAccel);

        // Print relative rotation and calculated torque
        //Debug.Log($"Relative rotation axis: {relRot.normalized}, angle: {relRot.magnitude} rad, torque: {torque}");

        return torque;
    }

    private Vector3 GetForceOnTracked()
    {
        // Evaluate the physics force against the tracked object's constant-velocity
        // trajectory from the previous timestep, then apply the equal and opposite
        // force to the tracked object.
        Vector3 constantVelocityPos =
            prevTrackedPos + prevTrackedVel * Time.fixedDeltaTime;
        return -ComputeForceOnPhysics(constantVelocityPos, prevTrackedVel);
    }
}
