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
    private Vector3 previousForceOnPhysics = Vector3.zero;
    private bool hasPreviousPhysicsStep;
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
    protected void FixedUpdate()
    {
        fixedStep++;
        UpdateMassProperties();

        // Update kinematics of the tracked object
        UpdateKinematics();
        // Calculate the force and torque on the physics object
        Vector3 newForceOnPhysics = ComputeForceOnPhysics();
        Vector3 newTorqueOnPhysics = ComputeTorqueOnPhysics();

        // Trims by max force
        if (newForceOnPhysics.magnitude > maxPhysicsForce)
        {
            newForceOnPhysics = newForceOnPhysics.normalized * maxPhysicsForce;
        }
        physicsAb.AddForce(newForceOnPhysics);

        // Trims by max torque
        if (newTorqueOnPhysics.magnitude > maxPhysicsTorque)
        {
            newTorqueOnPhysics = newTorqueOnPhysics.normalized * maxPhysicsTorque;
        }
        physicsAb.AddTorque(newTorqueOnPhysics);

        previousForceOnPhysics = newForceOnPhysics;
        hasPreviousPhysicsStep = true;

        // Gets inertia force to apply if applicable
        Vector3 inertiaForce = -physicsMass * trackedAccel;
        // Gets accurate force to apply on tracked object
        Vector3 trackedForce = GetForceOnTracked();
        UpdateWrench(trackedForce, newTorqueOnPhysics, inertiaForce);

        //LogDiagnostics(newForceOnPhysics, newTorqueOnPhysics, trackedForce, inertiaForce);
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
        return ComputeForceOnPhysics(trackedTransform.position);
    }

    private Vector3 ComputeForceOnPhysics(Vector3 trackedPosition)
    {
        if (posStiffness == 0)
        {
            return Vector3.zero;
        }

        Vector3 relPos = physicsAb.transform.position - trackedPosition;

        Vector3 accel =
            -(maxStiffness * posStiffness * relPos
            + maxDamping * posDamping * physicsAb.linearVelocity);
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

        Vector3 angAccel =
            -(maxStiffness * rotStiffness * relRot
            + maxDamping * rotDamping * physicsAngVel);
        // Apply the complete world-space inertia tensor. This preserves coupling
        // between axes when the aggregate tensor has products of inertia.
        Vector3 torque = Utils.MultiplyInertia(physicsInertia, angAccel);

        // Print relative rotation and calculated torque
        //Debug.Log($"Relative rotation axis: {relRot.normalized}, angle: {relRot.magnitude} rad, torque: {torque}");

        return torque;
    }

    private Vector3 GetForceOnTracked()
    {
        // Estimate the external force from the previous physics step:
        //
        //     F_external = m * acceleration - F_controller
        //
        // The controller force is subtracted so that a body with no external
        // force produces zero haptic force, even while it is being controlled.
        Vector3 externalForce =
            physicsMass * physicsAccel - previousForceOnPhysics;

        if (!hasPreviousPhysicsStep)
            return Vector3.zero;

        return externalForce;
    }

    private void LogDiagnostics(
        Vector3 forceOnPhysics,
        Vector3 torqueOnPhysics,
        Vector3 forceOnTracked,
        Vector3 inertiaForce)
    {
        float timeStep = Time.fixedDeltaTime;
        Vector3 relPos = physicsAb.transform.position - trackedTransform.position;
        Vector3 dampingVelocity = physicsVel;
        Vector3 controlAcceleration =
            -(maxStiffness * posStiffness * relPos
            + maxDamping * posDamping * dampingVelocity);
        float relativePositionDot =
            hasPreviousRelativePosition
                ? Vector3.Dot(previousRelativePosition, relPos)
                : 0f;

        Debug.Log(
            $"[{name}] Step={fixedStep}, t={Time.time:F4}, dt={timeStep:F6}\n" +
            $"tracked pos={trackedTransform.position}, prev={prevTrackedPos}, " +
            $"vel={trackedVel}, prevVel={prevTrackedVel}\n" +
            $"physics pos={physicsAb.transform.position}, vel={physicsVel}, " +
            $"accel={physicsAccel}, mass={physicsMass:F6}\n" +
            $"relative pos={relPos}, |pos|={relPos.magnitude:F6}, " +
            $"damping velocity={dampingVelocity}, " +
            $"|vel|={dampingVelocity.magnitude:F6}\n" +
            $"control accel={controlAcceleration}\n" +
            $"position dot previous={relativePositionDot:F6}\n" +
            $"force physics={forceOnPhysics}, |force|={forceOnPhysics.magnitude:F6}, " +
            $"force tracked={forceOnTracked}, |force|={forceOnTracked.magnitude:F6}\n" +
            $"inertia force={inertiaForce}, |inertia|={inertiaForce.magnitude:F6}, " +
            $"torque={torqueOnPhysics}, |torque|={torqueOnPhysics.magnitude:F6}");

        if (hasPreviousRelativePosition && relativePositionDot < 0f)
        {
            Debug.LogWarning(
                $"[{name}] Relative position crossed through zero; " +
                "this is an overshoot indicator.");
        }

        if (!IsFinite(forceOnPhysics) || !IsFinite(forceOnTracked) ||
            !IsFinite(inertiaForce) || !IsFinite(torqueOnPhysics))
        {
            Debug.LogError($"[{name}] Non-finite control output detected.");
        }

        if (forceOnPhysics.magnitude > maxPhysicsForce ||
            torqueOnPhysics.magnitude > maxPhysicsTorque)
        {
            Debug.LogWarning(
                $"[{name}] Control output exceeds configured limits. " +
                $"force={forceOnPhysics.magnitude:F6}/{maxPhysicsForce:F6}, " +
                $"torque={torqueOnPhysics.magnitude:F6}/{maxPhysicsTorque:F6}. " +
                "The runtime clamps are currently disabled.");
        }

        previousRelativePosition = relPos;
        hasPreviousRelativePosition = true;
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);    }
}
