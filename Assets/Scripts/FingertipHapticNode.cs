using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

public class FingertipHapticNode : HapticNode
{
    // Tracked node kinematics
    private Vector3 trackedPos = Vector3.zero;
    private Vector3 prevTrackedPos = Vector3.zero;
    private Vector3 trackedVel = Vector3.zero;
    private Vector3 trackedAccel = Vector3.zero;
    private Vector3 prevTrackedVel = Vector3.zero;

    // Mass of physics body
    public float physicsMass;

    public float maxForce = 20f;

    //Physics node kinematics
    private Vector3 physicsPos = Vector3.zero;
    private Vector3 prevPhysicsPos = Vector3.zero;
    private Vector3 physicsVel = Vector3.zero;
    private Vector3 prevPhysicsVel = Vector3.zero;
    private Vector3 physicsAccel = Vector3.zero;
    private Vector3 prevPhysicsForce = Vector3.zero;
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
        UpdateKinematics();
    }

    // Fixed update is called once per physics frame
    public void UpdatePhysics()
    {
        fixedStep++;

        // Update kinematics of the tracked object
        UpdateKinematics();
        // Calculate the force and torque on the physics object
        Vector3 newPhysicsForce = ComputeForceOnPhysics();
        
        // Trims by max force
        LastPhysicsForceMagnitude = newPhysicsForce.magnitude;
        LastPhysicsForceReachedLimit =
            maxForce > 0f &&
            newPhysicsForce.magnitude >= maxForce;
        if (newPhysicsForce.magnitude > maxForce)
        {
            newPhysicsForce = newPhysicsForce.normalized * maxForce;
        }

        // Updates haptic node state
        Vector3 inertiaForce = -physicsMass * physicsAccel;
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

        prevTrackedVel = trackedVel;
        trackedVel =
            (trackedPos - prevTrackedPos) / Time.fixedDeltaTime;
        trackedAccel = (trackedVel - prevTrackedVel) / Time.fixedDeltaTime;

        // Physics position kinematics
        prevPhysicsPos = physicsPos;
        physicsPos = gameObject.transform.position;
        prevPhysicsVel = physicsVel;
        physicsVel = (physicsPos - prevPhysicsPos) / Time.fixedDeltaTime;
        physicsAccel = (physicsVel - prevPhysicsVel) / Time.fixedDeltaTime;
    }

    private Vector3 ComputeForceOnPhysics()
    {
        if (posStiffness == 0)
        {
            return Vector3.zero;
        }

        Vector3 relPos = physicsPos - trackedPos;

        Vector3 relVel = physicsVel - trackedVel;

        Vector3 accel =
            -(Utils.GetMaxStiffness() * posStiffness * relPos
            + Utils.GetMaxDamping() * posDamping * relVel);
        Vector3 force = physicsMass * accel;
        //Print relative position and calculated force
        //Debug.Log($"Relative position: {relPos}, force: {force}");

        return force;
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
            $"physics pos={physicsPos:F4}, vel={physicsVel:F4}, " +
            $"accel={physicsAccel}, mass={physicsMass:F6}\n" +
            $"force physics={forceOnPhysics}, |force|={forceOnPhysics.magnitude:F6}, " +
            $"inertia force={inertiaForce}, |inertia|={inertiaForce.magnitude:F6}, " +
            $"torque={torqueOnPhysics}, |torque|={torqueOnPhysics.magnitude:F6}");
    }
}
