using System;
using System.Collections.Generic;
using UnityEngine;

public class HapticNode : MonoBehaviour
{
    private static readonly List<HapticNode> activeNodes = new List<HapticNode>();

    // HapticRenderClient script
    public HapticRenderClient haptics;

    // Tracked object
    public Transform trackedTransform;

    // Haptic node to reference inertia from
    public HapticNode inertiaReference;

    [Range(0f, 1f)]
    public float posStiffness;
    [Range(0f, 1f)]
    public float posDamping;
    [Range(0f, 1f)]
    public float rotStiffness;
    [Range(0f, 1f)]
    public float rotDamping;
    public float minOutputForce;
    public float maxPhysicsForce;
    public float maxPhysicsTorque;

    // Force visualization object
    private GameObject forceVisual;
    
    // Whether this node is for reference only
    public bool isReferenceOnly;

    // Target to send inertia reference to if relevant
    public HapticNode inertiaTarget;

    // Inertial force on node
    public Vector3 inertiaForce;

    // Linear momentum of node
    public Vector3 linearMomentum;
    private bool physicsUpdated = false;

    // Linear momentum on reference object
    public Vector3 linearMomentumReference;
    private bool referenceUpdated = false;

    public static IReadOnlyList<HapticNode> ActiveNodes => activeNodes;

    void OnEnable()
    {
        if (!activeNodes.Contains(this))
            activeNodes.Add(this);
    }

    void OnDisable()
    {
        activeNodes.Remove(this);
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    protected void Start()
    {
        if (inertiaReference != null)
        {
            inertiaReference.SetInertiaTarget(this);
        }

        // Spawn force visual on tracked transform
        if (trackedTransform == null)
        {
            Debug.LogError($"[{name}] HapticNode requires a tracked transform.");
            return;
        }

        if (haptics.forceVisualPrefab == null)
        {
            Debug.LogError($"[{name}] HapticRenderClient requires a force visual prefab.");
            return;
        }

        forceVisual = Instantiate(haptics.forceVisualPrefab, trackedTransform.position, trackedTransform.rotation);
        forceVisual.transform.SetParent(trackedTransform, true);
        forceVisual.SetActive(false);
    }

    protected void SetInertiaTarget(HapticNode target)
    {
        inertiaTarget = target;
    }

    // Usually called from the physics update loop to update the wrench on the physics object
    public void UpdatePhysicsState(Vector3 linearMomentum, Vector3 inertiaForce)
    {
        this.linearMomentum = linearMomentum;
        this.inertiaForce = inertiaForce;
        physicsUpdated = true;

        // Update inertia target if it exists
        if (inertiaTarget != null)
        {
            inertiaTarget.UpdatePhysicsState(linearMomentum, inertiaForce);
        }

        // If the reference object has been updated, update the state
        if (inertiaReference == null || referenceUpdated)
        {
            UpdateState();
        }
    }

    public void UpdateInertiaState(Vector3 linearMomentumReference)
    {
        this.linearMomentumReference = linearMomentumReference;
        referenceUpdated = true;

        // If the physics object has been updated, update the state
        if (physicsUpdated)
        {
            UpdateState();
        }
    }

    private void UpdateState()
    {
        // If this node is reference only, skip updating the state
        if (isReferenceOnly)
        {
            return;
        }

        // Calculate the force to emulate on the tracked object
        Vector3 force = (linearMomentum - linearMomentumReference)/Time.fixedDeltaTime;

        // If inertia is enabled, adds inertial force to the tracked
        if (haptics.useInertia)
        {
            force += inertiaForce;
        }

        // Clamps force to minimum and maximum
        if (force.magnitude < haptics.minOutputForce)
        {
            force = Vector3.zero;
        }

        // Updates force visualization
        UpdateForceVisual(force);
    }

    private void UpdateForceVisual(Vector3 force)
    {
        // If visualization is on and a force is present
        if (haptics.visualization && force.magnitude > 0)
        {
            forceVisual.SetActive(true);
            //Scales and orients force visual by force
            forceVisual.transform.rotation = Quaternion.FromToRotation(Vector3.up, force.normalized);
            forceVisual.transform.localScale = new Vector3(1f, force.magnitude * haptics.forceVisualScale, 1f);
        }
        else
        {
            forceVisual.SetActive(false);
        }
    }
}
