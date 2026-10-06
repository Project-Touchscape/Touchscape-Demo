using UnityEngine;
using UnityEngine.XR.Hands;
using System.Collections.Generic;

public class HandSkeleton : MonoBehaviour
{
    public HapticRenderClient haptics;
    public GameObject palm;
    public Transform trackedPalm;
    public GameObject[] initialFingerJoints = new GameObject[N_FINGERS];
    public Transform[] trackedJoints = new Transform[N_FINGERS];

    [Range(0.1f, 10f)]
    public float strength = 1f;

    [SerializeField]
    [Tooltip("The mass of each finger bone; the palm will be 3x this.")]
    public float perBoneMass = 3.0f;

    [SerializeField]
    [Tooltip("The width of each finger bone.")]
    public float boneWidth = 0.016f;

    [SerializeField]
    [Tooltip("The physics material that the hand uses.")]
    public PhysicsMaterial material = null;

    public bool useGravity = false;

    // Cache previous gravity state to detect changes
    private bool prevGravity = false;
    private const int N_FINGERS = 5;
    private const int N_ACTIVE_BONES = 3;
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

        rootBody.mass = perBoneMass * 3f;
        rootBody.immovable = false;
        rootBody.useGravity = useGravity;
        rootBody.solverIterations = 60;
        rootBody.solverVelocityIterations = 20;
        articulationBodies.Add(rootBody);

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

        // Constructs haptic node at palm
        if (!palm.TryGetComponent<HapticNode>(out var hapticNode))
            hapticNode = palm.AddComponent<HapticNode>();
        hapticNode.haptics = haptics;
        hapticNode.trackedTransform = trackedPalm;
    }

    private void ConstructFingers()
    {
        // Each entry is the first joint GameObject for one finger. The remaining joints
        // are found by walking the existing GameObject hierarchy.
        if (initialFingerJoints == null)
            return;

        for (int fingerIndex = 0; fingerIndex < Mathf.Min(N_FINGERS, initialFingerJoints.Length); fingerIndex++)
        {
            GameObject currentJoint = initialFingerJoints[fingerIndex];
            for (int jointIndex = 0; jointIndex < N_ACTIVE_BONES; jointIndex++)
            {
                if (currentJoint == null)
                    break;

                GameObject nextJoint = GetNextJoint(currentJoint);
                ConstructFingerJoint(currentJoint, nextJoint);
                currentJoint = nextJoint;
            }

            // Constructs haptic node at fingertip
            ConstructHapticNode(fingerIndex, currentJoint);
        }
    }

    private GameObject GetNextJoint(GameObject currentJoint)
    {
        if (currentJoint == null || currentJoint.transform.childCount == 0)
            return null;

        return currentJoint.transform.GetChild(0).gameObject;
    }

    private void ConstructFingerJoint(GameObject currentJoint, GameObject nextJoint)
    {
        // Components belong to the existing tracked joint object. No proxy GameObject is needed.
        if (!currentJoint.TryGetComponent<ArticulationBody>(out var body))
            body = currentJoint.AddComponent<ArticulationBody>();

        CapsuleCollider capsule = currentJoint.GetComponent<CapsuleCollider>();
        if (capsule == null)
            capsule = currentJoint.AddComponent<CapsuleCollider>();

        float boneLength = nextJoint == null
            ? 0.02f
            : Vector3.Distance(currentJoint.transform.position, nextJoint.transform.position);

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
        body.twistLock = ArticulationDofLock.FreeMotion;
        body.xDrive = new ArticulationDrive
        {
            stiffness = 100f * strength,
            forceLimit = 1000f * strength,
            damping = 3f,
            lowerLimit = -10f,
            upperLimit = 89f
        };
        articulationBodies.Add(body);
    }

    private void ConstructHapticNode(int fingerIndex, GameObject fingertipJoint)
    {
        if (fingertipJoint == null)
            return;

        // Create ArticulatedBody for fingertip joint if it doesn't exist
        if (!fingertipJoint.TryGetComponent<ArticulationBody>(out var ab))
            ab = fingertipJoint.AddComponent<ArticulationBody>();
        
        ab.mass = perBoneMass;
        ab.useGravity = useGravity;
        ab.jointType = ArticulationJointType.FixedJoint;
        ab.solverIterations = 60;
        ab.solverVelocityIterations = 20;
        articulationBodies.Add(ab);

        // Create haptic node
        if (!fingertipJoint.TryGetComponent<HapticNode>(out var hapticNode))
            hapticNode = fingertipJoint.AddComponent<HapticNode>();

        hapticNode.haptics = haptics;
        hapticNode.trackedTransform = trackedJoints[fingerIndex];
    }
}
