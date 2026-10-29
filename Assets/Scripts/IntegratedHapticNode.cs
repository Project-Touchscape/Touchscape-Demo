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

    //Tracked position
    private Vector3 trackedPos = Vector3.zero;

    //Tracked orientation
    private Quaternion trackedRot = Quaternion.identity;

    //Tracked velocity
    private Vector3 trackedVel = Vector3.zero;

    //Tracked acceleration
    private Vector3 trackedAccel = Vector3.zero;

    // Kinematics of the tracked object's local center of mass.
    private Vector3 trackedCOMPos = Vector3.zero;
    private Vector3 trackedCOMVel = Vector3.zero;
    private Vector3 trackedCOMAccel = Vector3.zero;

    // ArticulationBody of the physics object
    private ArticulationBody physicsAb;

    // Mass of articulation body
    private float physicsMass;

    // Center of mass of articulation body (world space)
    private Vector3 physicsCOM;

    // Intertial matrix of articulation body (world space)
    private Matrix4x4 physicsInertia;

    // Velocity of the equivalent body's center of mass.
    private Vector3 physicsCOMVelocity = Vector3.zero;
    private Vector3 prevPhysicsCOMVel = Vector3.zero;

    // Center of mass relative to the root transform.
    private Vector3 localCOM = Vector3.zero;

    // Force to emulate with tracked object
    private Vector3 forceOnTracked = Vector3.zero;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    new protected void Start()
    {
        // Initializes haptic node
        base.Start();

        // Gets physics object rigidbody
        physicsAb = gameObject.GetComponent<ArticulationBody>();
        UpdateMassProperties();

        // Initializes tracked object's kinematics
        if (trackedTransform != null)
        {
            trackedPos = trackedTransform.position;
            trackedRot = trackedTransform.rotation;
            trackedCOMPos = trackedTransform.TransformPoint(localCOM);
        }
    }

    private void SetInitialMassProperties()
    {
        physicsMass = physicsAb.mass;
        physicsCOM = physicsAb.worldCenterOfMass;
        physicsInertia = Utils.WorldInertiaMatrix(physicsAb);
        localCOM = physicsAb.transform.InverseTransformPoint(physicsCOM);
    }

    public void SetChainedMassProperties()
    {
        Utils.GetChainedMassProperties(physicsAb, out float mass, out Vector3 centerOfMass, out Matrix4x4 inertia);
        physicsMass = mass;
        physicsCOM = centerOfMass;
        physicsInertia = inertia;
        localCOM = physicsAb.transform.InverseTransformPoint(physicsCOM);
    }

    private void UpdateMassProperties()
    {
        if (useChainedMass)
            SetChainedMassProperties();
        else
            SetInitialMassProperties();

        physicsCOMVelocity = physicsAb.linearVelocity +
            Vector3.Cross(physicsAb.angularVelocity, physicsCOM - physicsAb.worldCenterOfMass);
    }

    new protected Vector3 GetInertialForce()
    {
        return -physicsMass * trackedCOMAccel;
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
        // Trims by max force
        if (newForceOnPhysics.magnitude > maxForce)
        {
            newForceOnPhysics = newForceOnPhysics.normalized * maxForce;
        }
        physicsAb.AddForce(newForceOnPhysics);

        // Trims by max torque
        if (newTorqueOnPhysics.magnitude > maxTorque)
        {
            newTorqueOnPhysics = newTorqueOnPhysics.normalized * maxTorque;
        }
        physicsAb.AddTorque(newTorqueOnPhysics);

        // Gets inertia force to apply if applicable
        Vector3 inertiaForce = -physicsMass * trackedCOMAccel;
        // Gets spring component of the force on the physics object
        Vector3 springComponent = GetSpringComponent();
        UpdateWrench(newForceOnPhysics, newTorqueOnPhysics, springComponent,inertiaForce);

        // Updates previous physics velocity
        prevPhysicsCOMVel = physicsCOMVelocity;
    }

    // Update is called once per frame
    void Update()
    {
    }

    private void UpdateKinematics()
    {
        // Position error remains based on the GameObject origin.
        Vector3 trackedPosCopy = trackedTransform.position;
        Vector3 prevTrackedPos = trackedPos;
        Vector3 currTrackedVel = (trackedPosCopy - prevTrackedPos) / Time.fixedDeltaTime;
        trackedAccel = (currTrackedVel - trackedVel) / Time.fixedDeltaTime;

        // Velocity and acceleration use the local center of mass.
        Vector3 trackedCOMCopy = trackedTransform.TransformPoint(localCOM);
        Vector3 currTrackedCOMVel = (trackedCOMCopy - trackedCOMPos) / Time.fixedDeltaTime;
        trackedCOMAccel = (currTrackedCOMVel - trackedCOMVel) / Time.fixedDeltaTime;
        trackedCOMPos = trackedCOMCopy;
        trackedCOMVel = currTrackedCOMVel;

        trackedPos = trackedPosCopy;
        trackedRot = trackedTransform.rotation;
        trackedVel = currTrackedVel;
    }

    private Vector3 ComputeForceOnPhysics()
    {
        if (posStiffness == 0 && posDamping == 0)
        {
            return Vector3.zero;
        }
        // Gets relative position and velocities of tracked and physics objects
        Vector3 trackedPosCopy = trackedPos;
        Vector3 physicsPos = gameObject.transform.position;

        Vector3 physicsVel = physicsCOMVelocity - trackedCOMVel;

        Vector3 relPos = physicsPos - trackedPosCopy;

        // Applies coefficients
        Vector3 accel = -(maxStiffness * posStiffness * relPos + maxDamping * posDamping * physicsVel);
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
        Quaternion trackedRotCopy = trackedRot;
        Quaternion physicsRot = gameObject.transform.rotation;

        Vector3 physicsAngVel = physicsAb.angularVelocity;

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

    private Vector3 GetSpringComponent()
    {
        if (posStiffness == 0 && posDamping == 0)
        {
            return Vector3.zero;
        }
        //Gets previous tracked and physics positions
        Vector3 trackedPosCopy = trackedPos;
        Vector3 prevTrackedPos = trackedPosCopy - trackedVel * Time.fixedDeltaTime;
        Vector3 prevPhysicsPos = gameObject.transform.position;

        //Gets predicted current physics position and velocity
        Vector3 relativeCOMVelocity = prevPhysicsCOMVel - trackedCOMVel;
        Vector3 predPhysicsVel = -posStiffness * (prevPhysicsPos - prevTrackedPos) / Time.fixedDeltaTime + relativeCOMVelocity * (1 - posDamping);
        Vector3 predPhysicsPos = (prevTrackedPos - prevPhysicsPos) * posStiffness + prevPhysicsPos + relativeCOMVelocity * Time.fixedDeltaTime * (1 - posDamping);

        //Gets predicted (inertial) spring force
        Vector3 predictedAccel = -(maxStiffness * posStiffness * (predPhysicsPos - trackedPosCopy) + maxDamping * posDamping * predPhysicsVel);
        Vector3 predictedForce = physicsMass * predictedAccel;
        return predictedForce;
    }
}
