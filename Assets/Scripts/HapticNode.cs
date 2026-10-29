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

    public bool useInertia;
    public float inertiaCorrection = 0.5f;
    [Range(0f, 1f)]
    public float posStiffness;
    public float posDamping;
    [Range(0f, 1f)]
    public float rotStiffness;
    public float rotDamping;
    public float minOutputForce;
    public float maxForce;
    public float maxTorque;

    // Force visualization object
    private GameObject forceVisual;

    // Maximum stiffness and damping
    protected float maxStiffness;
    protected float maxDamping;

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
        if (haptics != null)
            haptics.RegisterNode(this);

        // Gets maximum stiffness and damping coefficients (would bring object to rest in one frame)
        maxStiffness = 1 / Mathf.Pow(Time.fixedDeltaTime, 2);
        maxDamping = 1 / Time.fixedDeltaTime;

        // Spawn force visual on tracked transform
        forceVisual = Instantiate(haptics.forceVisualPrefab, trackedTransform.position, trackedTransform.rotation);
        forceVisual.transform.SetParent(trackedTransform, true);
        forceVisual.SetActive(false);
    }

    // Usually called from the physics update loop to update the wrench on the physics object
    public void UpdateWrench(Vector3 forceOnTracked, Vector3 torqueOnTracked, Vector3 inertiaForce)
    {

        // Calculate the force to emulate on the tracked object
        Vector3 force = forceOnTracked;
        force += inertiaForce*inertiaCorrection;

        // If inertia is enabled, adds inertial force to the tracked
        if (useInertia)
        {
            force += inertiaForce;
        }

        Debug.Log($"Force on tracked: {force}, Inertia force: {inertiaForce}");
        
        // Clamps force to minimum
        if (force.magnitude < minOutputForce)
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
