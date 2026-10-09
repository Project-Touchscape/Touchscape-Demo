using UnityEngine;
using UnityEngine.XR.Hands;
using System.Collections.Generic;

public class HandSkeleton : MonoBehaviour
{
    private const int N_FINGERS = 5;
    private const int N_ACTIVE_BONES = 3;

    public HapticRenderClient haptics;
    public GameObject palm;
    public Transform trackedRoot;
    public GameObject[] initialJoints = new GameObject[N_FINGERS];
    public Transform[] trackedJoints = new Transform[N_FINGERS];
    public float[] fingerBoneWidths = new float[3] {0.016f, 0.016f, 0.016f};
    public float[] thumbBoneWidths = new float[3] {0.016f, 0.016f, 0.016f};

    [SerializeField]
    [Tooltip("The mass of each finger bone")]
    public float perBoneMass = 0.1f;
    public float palmMass = 0.3f;

    [SerializeField]
    [Tooltip("The physics material that the hand uses.")]
    public PhysicsMaterial material = null;

    public bool useGravity = false;

    // Cache previous gravity state to detect changes
    private bool prevGravity = false;

    private XRHand hand;
    private List<ArticulationBody> articulationBodies = new List<ArticulationBody>();

    void Start()
    {
        // Set initial gravity state
        prevGravity = useGravity;
        
        // Initialize palm articulation body and collider
        ConstructPalm();
        ConstructFingers();
    }

    void Update()
    {
        // Check if the gravity state has changed
        if (prevGravity != useGravity)
        {
            prevGravity = useGravity;
            // Update the gravity state of the hand
            foreach (var body in articulationBodies)
            {
                if (body != null)
                {
                    body.useGravity = useGravity;
                }
            }
        }
    }

    private void ConstructPalm()
    {
        // Adds root articulation body to current body to preserve hierarchy
        ArticulationBody rootBody = gameObject.GetComponent<ArticulationBody>();
        if (rootBody == null)
        {
            rootBody = gameObject.AddComponent<ArticulationBody>();
        }

        rootBody.mass = palmMass;
        rootBody.immovable = false;
        rootBody.useGravity = useGravity;
        rootBody.solverIterations = 60;
        rootBody.solverVelocityIterations = 20;
        articulationBodies.Add(rootBody);

        // Constructs haptic node at root 
        if (!gameObject.TryGetComponent<HapticNode>(out var hapticNode))
            hapticNode = gameObject.AddComponent<HapticNode>();
        hapticNode.haptics = haptics;
        hapticNode.trackedTransform = trackedRoot;
        hapticNode.useChainedMass = true;

        // Adds fixed articulation body to palm object
        ArticulationBody palmBody = palm.GetComponent<ArticulationBody>();
        if (palmBody == null)
        {
            palmBody = palm.AddComponent<ArticulationBody>();
        }
        palmBody.mass = perBoneMass * 3f;
        palmBody.useGravity = useGravity;
        palmBody.solverIterations = 60;
        palmBody.solverVelocityIterations = 20;
        palmBody.jointType = ArticulationJointType.FixedJoint;
        articulationBodies.Add(palmBody);

        // Reuse the first child as the palm collider object, creating one when the palm
        // does not already have a child. The collider must not be added to the palm root,
        // because the root owns the articulation joint.
        GameObject palmChild;
        if (palm.transform.childCount > 0)
        {
            palmChild = palm.transform.GetChild(0).gameObject;
        }
        else
        {
            palmChild = new GameObject(palm.name + " Collider");
            palmChild.transform.SetParent(palm.transform, false);
            palmChild.layer = palm.layer;
        }

        BoxCollider palmCollider = palmChild.GetComponent<BoxCollider>();
        if (palmCollider == null)
        {
            palmCollider = palmChild.AddComponent<BoxCollider>();
            palmCollider.center = new Vector3(0f, 0.005f, -0.015f);
            palmCollider.size = new Vector3(0.06f, 0.02f, 0.07f);
        }

        // Always apply the configured hand material, including when the collider
        // was already present on the child.
        palmCollider.material = material;
    }

    private void ConstructFingers()
    {
        // Each entry is the first joint GameObject for one finger. The remaining joints
        // are found by walking the existing GameObject hierarchy.
        if (initialJoints == null)
            return;

        for (int fingerIndex = 0; fingerIndex < Mathf.Min(N_FINGERS, initialJoints.Length); fingerIndex++)
        {
            GameObject currentJoint = initialJoints[fingerIndex];
            for (int jointIndex = 0; jointIndex < N_ACTIVE_BONES; jointIndex++)
            {
                if (currentJoint == null)
                    break;

                GameObject nextJoint = GetNextJoint(currentJoint);
                ConstructFingerJoint(fingerIndex, jointIndex, currentJoint, nextJoint);
                currentJoint = nextJoint;
            }
        }
    }

    private GameObject GetNextJoint(GameObject currentJoint)
    {
        if (currentJoint == null || currentJoint.transform.childCount == 0)
            return null;

        return currentJoint.transform.GetChild(0).gameObject;
    }

    private void ConstructFingerJoint(int fingerIndex, int jointIndex, GameObject currentJoint, GameObject nextJoint)
    {
        // Keep the articulation body on the tracked joint object.
        if (!currentJoint.TryGetComponent<ArticulationBody>(out var body))
            body = currentJoint.AddComponent<ArticulationBody>();

        // Put the collider on a child so its rotation can be set independently of the
        // tracked joint's parent orientation.
        const string colliderName = "Finger Collider";
        Transform colliderTransform = currentJoint.transform.Find(colliderName);
        if (colliderTransform == null)
        {
            GameObject colliderObject = new GameObject(colliderName);
            colliderTransform = colliderObject.transform;
            colliderTransform.SetParent(currentJoint.transform, false);
        }

        // Points child object z-axis in the direction of the next joint, or forward if there is no next joint.
        colliderTransform.localPosition = Vector3.zero;
        Vector3 boneDirection = nextJoint == null
            ? currentJoint.transform.forward
            : nextJoint.transform.position - currentJoint.transform.position;
        if (boneDirection.sqrMagnitude > Mathf.Epsilon)
            colliderTransform.rotation = Quaternion.LookRotation(boneDirection.normalized, currentJoint.transform.up);

        // Adds capsule collider to the child object
        CapsuleCollider capsule = colliderTransform.GetComponent<CapsuleCollider>();
        if (capsule == null)
            capsule = colliderTransform.gameObject.AddComponent<CapsuleCollider>();
        
        // Gets bone width by per joint specifcations
        float boneWidth = fingerIndex == N_FINGERS-1 ? thumbBoneWidths[jointIndex] : fingerBoneWidths[jointIndex];

        // Gets bone length from distance to the next joint
        float boneLength = nextJoint == null ? 0.02f : Vector3.Distance(currentJoint.transform.position, nextJoint.transform.position);
        
        // Shortens the bone length for the last joint to be tangent to the fingertip
        if (jointIndex == N_ACTIVE_BONES - 1)
            boneLength -= boneWidth * 0.5f;
        
        capsule.direction = 2;
        capsule.radius = boneWidth * 0.5f;
        capsule.height = boneLength + boneWidth;
        capsule.center = new Vector3(0f, 0f, boneLength * 0.5f);
        capsule.material = material;

        body.mass = perBoneMass;
        body.useGravity = useGravity;
        body.anchorPosition = Vector3.zero;
        body.anchorRotation = Quaternion.identity;
        body.solverIterations = 60;
        body.solverVelocityIterations = 20;
        body.jointType = ArticulationJointType.RevoluteJoint;
        body.twistLock = ArticulationDofLock.LimitedMotion;
        body.xDrive = new ArticulationDrive
        {
            stiffness = 1000f,
            forceLimit = 1000f,
            damping = 100f,
            lowerLimit = -10f,
            upperLimit = 89f
        };
        articulationBodies.Add(body);
    }
}
