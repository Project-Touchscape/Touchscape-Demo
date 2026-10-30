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
    private bool initialized;

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
        Initialize();
    }

    private void Initialize()
    {
        if (initialized)
            return;

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
        initialized = true;
    }

    // Usually called from the physics update loop to update the wrench on the physics object
    public void UpdateWrench(Vector3 forceOnTracked, Vector3 torqueOnTracked, Vector3 inertiaForce)
    {
        Initialize();

        // Calculate the force to emulate on the tracked object
        Vector3 force = forceOnTracked;

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
        if (!initialized)
            return;

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
