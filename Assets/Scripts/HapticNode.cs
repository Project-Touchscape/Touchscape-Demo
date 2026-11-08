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
    private readonly List<Collider> attachedColliders = new List<Collider>();

    [Range(0f, 1f)]
    public float posStiffness;
    [Range(0f, 1f)]
    public float posDamping;
    [Range(0f, 1f)]
    public float rotStiffness;
    [Range(0f, 1f)]
    public float rotDamping;
    public float maxPhysicsForce;
    public float maxPhysicsTorque;

    // Force visualization object
    private GameObject forceVisual;
    private readonly List<HapticCollisionHandler> collisionHandlers =
        new List<HapticCollisionHandler>();
    private bool initialized;

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
        ConfigureCollisionHandler();
        initialized = true;
    }

    public bool HasActiveCollision =>
        collisionHandlers.Exists(handler => handler.HasActiveCollision);

    public void SetAttachedCollider(Collider collider)
    {
        SetAttachedColliders(collider == null ? null : new List<Collider> { collider });
    }

    public void SetAttachedColliders(IReadOnlyList<Collider> colliders)
    {
        attachedColliders.Clear();
        if (colliders != null)
        {
            for (int index = 0; index < colliders.Count; index++)
            {
                if (colliders[index] != null &&
                    !attachedColliders.Contains(colliders[index]))
                {
                    attachedColliders.Add(colliders[index]);
                }
            }
        }

        if (initialized)
            ConfigureCollisionHandler();
    }

    private void ConfigureCollisionHandler()
    {
        collisionHandlers.Clear();

        for (int index = 0; index < attachedColliders.Count; index++)
        {
            Collider collider = attachedColliders[index];
            AddCollisionHandler(collider.gameObject);

            ArticulationBody body =
                collider.GetComponentInParent<ArticulationBody>();
            if (body != null)
                AddCollisionHandler(body.gameObject);
        }
    }

    private void AddCollisionHandler(GameObject target)
    {
        HapticCollisionHandler handler =
            target.GetComponent<HapticCollisionHandler>();
        if (handler == null)
            handler = target.AddComponent<HapticCollisionHandler>();

        if (!collisionHandlers.Contains(handler))
            collisionHandlers.Add(handler);
    }

    // Usually called from the physics update loop to update the wrench on the physics object
    public void UpdateState(Vector3 physicsForce, Vector3 inertiaForce)
    {
        Initialize();
        
        // If inertia is disabled, subtract the inertia force from the physics force
        if (!haptics.useInertia) {
            physicsForce -= inertiaForce;
        }

        // Only uses non-inertial force if there is an active collision
        if (haptics.collisionFilter && !HasActiveCollision)
        {
            physicsForce = Vector3.zero;
        }

        // Clamps force to minimum
        if (physicsForce.magnitude < haptics.minOutputForce)
        {
            physicsForce = Vector3.zero;
        }

        // Updates force visualization
        UpdateForceVisual(physicsForce);
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

    public sealed class HapticCollisionHandler : MonoBehaviour
    {
        private readonly HashSet<int> activeCollisions = new HashSet<int>();

        public bool HasActiveCollision => activeCollisions.Count > 0;

        private void OnCollisionEnter(Collision collision)
        {
            Debug.Log($"[{name}] Collision enter with {collision.collider.name}");
            activeCollisions.Add(collision.collider.GetInstanceID());
        }

        private void OnCollisionExit(Collision collision)
        {
            activeCollisions.Remove(collision.collider.GetInstanceID());
        }

        private void OnTriggerEnter(Collider other)
        {
            activeCollisions.Add(other.GetInstanceID());
        }

        private void OnTriggerExit(Collider other)
        {
            activeCollisions.Remove(other.GetInstanceID());
        }

        private void OnDisable()
        {
            activeCollisions.Clear();
        }
    }
}
