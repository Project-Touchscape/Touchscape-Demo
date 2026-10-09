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
    public GameObject[] fingers = new GameObject[N_FINGERS];
    public Transform[] trackedFingers = new Transform[N_FINGERS];
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
    public float fingerStiffness = 1000f;
    public float fingerDamping = 100f;

    // Cache previous gravity state to detect changes
    private bool prevGravity = false;

    private XRHand hand;
    private List<ArticulationBody> articulationBodies = new List<ArticulationBody>();
    private List<float> initialJointAngles = new List<float>();

    // First finger joint is fixed metacarpal, so it doesn't have a collider
    private bool[] jointsWithColliders = new bool[4] {false, true, true, true};
    // First finger joint is fixed, second has splay, third and fourth do not have splay
    private int[] perJointDOF = new int[4] {0, 2, 1, 1};

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

    void FixedUpdate()
    {
        int jointID = 0;
        // Update the articulation bodies' target angles based on the tracked joint rotations
        for (int fingerIndex = 0; fingerIndex < Mathf.Min(N_FINGERS, fingers.Length); fingerIndex++)
        {
            GameObject currentJoint = fingers[fingerIndex];
            // Get the corresponding tracked joint transform
            Transform trackedJoint = trackedFingers[fingerIndex];
            if (trackedJoint == null || currentJoint == null)
                break;
            for (int jointIndex = 0; jointIndex < N_ACTIVE_BONES; jointIndex++)
            {
                // Get the degrees of freedom for the current joint
                int jointDOF = fingerIndex == N_FINGERS - 1 ? perJointDOF[jointIndex + 1] : perJointDOF[jointIndex];
                if (jointDOF > 0)
                {
                    // Update the articulation body's target angle based on the tracked joint's rotation
                    UpdateJointTarget(jointID, jointIndex, currentJoint, trackedJoint);
                }

                // Move to the next joint in the finger hierarchy
                currentJoint = currentJoint.transform.GetChild(0).gameObject;
                trackedJoint = trackedJoint.GetChild(0);
                jointID++;
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
        if (fingers == null)
            return;

        for (int fingerIndex = 0; fingerIndex < Mathf.Min(N_FINGERS, fingers.Length); fingerIndex++)
        {
            GameObject currentJoint = fingers[fingerIndex];
            for (int jointIndex = 0; jointIndex < N_ACTIVE_BONES; jointIndex++)
            {
                if (currentJoint == null)
                    break;

                ConstructFingerJoint(fingerIndex, jointIndex, currentJoint);
                currentJoint = currentJoint.transform.GetChild(0).gameObject;
            }
        }
    }

    private void ConstructFingerJoint(int fingerIndex, int jointIndex, GameObject currentJoint)
    {
        // Fetch per joint properties
        bool isThumb = fingerIndex == N_FINGERS - 1;
        float boneWidth = isThumb ? thumbBoneWidths[jointIndex] : fingerBoneWidths[jointIndex];
        // Thumb has one less joint so skip first one
        int effectiveIndex = isThumb ? jointIndex + 1 : jointIndex;
        bool useCollider = jointsWithColliders[effectiveIndex];
        int dof = perJointDOF[effectiveIndex];

        Vector3 boneDirection = GetBoneVector(currentJoint.transform);

        if (useCollider) {
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
            if (boneDirection.sqrMagnitude > Mathf.Epsilon)
                colliderTransform.rotation = Quaternion.LookRotation(boneDirection.normalized, currentJoint.transform.up);

            // Adds capsule collider to the child object
            CapsuleCollider capsule = colliderTransform.GetComponent<CapsuleCollider>();
            if (capsule == null)
                capsule = colliderTransform.gameObject.AddComponent<CapsuleCollider>();

            // Gets bone length from distance to the next joint
            float boneLength = boneDirection.magnitude;
            
            // Shortens the bone length for the last joint to be tangent to the fingertip
            if (jointIndex == N_ACTIVE_BONES - 1)
                boneLength -= boneWidth * 0.5f;
            
            capsule.direction = 2;
            capsule.radius = boneWidth * 0.5f;
            capsule.height = boneLength + boneWidth;
            capsule.center = new Vector3(0f, 0f, boneLength * 0.5f);
            capsule.material = material;
        }

        // Keep the articulation body on the tracked joint object.
        if (!currentJoint.TryGetComponent<ArticulationBody>(out var body))
            body = currentJoint.AddComponent<ArticulationBody>();

        // Gets initial joint angle using current bone direction relative to parent bone direction (X-axis rotation)
        float initialAngle = 0f;
        if (effectiveIndex > 0 && currentJoint.transform.parent != null)
        {
            Vector3 parentBoneDirection = GetBoneVector(currentJoint.transform.parent);
            initialAngle = Vector3.SignedAngle(parentBoneDirection, boneDirection, currentJoint.transform.right);
        }
        initialJointAngles.Add(initialAngle);

        body.mass = perBoneMass;
        body.useGravity = useGravity;
        body.anchorPosition = Vector3.zero;
        // Anchor rotations are expressed in the articulation body's local frame.
        // Convert the bone direction to that frame before aligning the anchor's
        // local forward axis with the bone.
        Vector3 localBoneDirection = currentJoint.transform.InverseTransformDirection(boneDirection.normalized);
        body.anchorRotation = Quaternion.FromToRotation(Vector3.forward, localBoneDirection);
        body.solverIterations = 60;
        body.solverVelocityIterations = 20;
        if (dof > 0)
        {
            body.jointType = ArticulationJointType.RevoluteJoint;
            body.twistLock = ArticulationDofLock.LimitedMotion;
            body.xDrive = new ArticulationDrive
            {
                // Note: positive angles are flexion, negative angles are extension. The initial angle is subtracted from the target to make the joint's current position the zero point for the drive.
                driveType = ArticulationDriveType.Force,
                target = -initialAngle,
                stiffness = fingerStiffness,
                forceLimit = 1000f,
                damping = fingerDamping,
                lowerLimit = -10f - initialAngle,
                upperLimit = 89f - initialAngle,
            };
        }
        else
        {
            body.jointType = ArticulationJointType.FixedJoint;
        }
        articulationBodies.Add(body);
    }

    /// <summary>
    /// Gets the direction of the bone from the current joint to the next joint.
    /// </summary>
    /// <param name="currentJoint">The current joint.</param>
    /// <returns>The direction of the bone.</returns>
    private Vector3 GetBoneVector(Transform currentJoint)
    {
        Transform nextJoint = currentJoint.childCount > 0 ? currentJoint.GetChild(0) : null;
        if (nextJoint == null)
            return currentJoint.forward;

        return nextJoint.position - currentJoint.position;
    }

    /// <summary>
    /// Updates the target angle of a joint based on the tracked joint's position.
    /// </summary>
    /// <param name="jointID">The ID of the joint to update.</param>
    /// <param name="jointIndex">The index of the joint to update.</param>
    /// <param name="currentJoint">The current joint.</param>
    /// <param name="trackedJoint">The tracked joint.</param>
    /// <param name="boneDirection">The direction of the bone.</param>
    /// <param name="boneUp">The up vector of the bone.</param>
    /// <returns>The direction of the bone.</returns>
    private void UpdateJointTarget(int jointID, int jointIndex, GameObject currentJoint, Transform trackedJoint)
    {
        if (currentJoint == null || trackedJoint == null)
            return;

        ArticulationBody body = currentJoint.GetComponent<ArticulationBody>();
        if (body != null && body.jointType == ArticulationJointType.RevoluteJoint)
        {
            // Measure the tracked bones around the physics joint's local X axis. The
            // articulation drive rotates around this axis, and using trackedJoint.right
            // here can introduce a thumb flexion offset when the two rigs use different
            // local axes.
            float initialAngle = initialJointAngles[jointID];
            Vector3 trackedBoneDirection = GetBoneVector(trackedJoint);
            Vector3 parentBoneDirection = GetBoneVector(trackedJoint.parent);
            // Note: positive angles are flexion, negative angles are extension. The initial angle is subtracted from the target to make the joint's current position the zero point for the drive.
            float currentAngle = Vector3.SignedAngle(parentBoneDirection, trackedBoneDirection, currentJoint.transform.right);
            float targetAngle = currentAngle - initialAngle;
            var drive = body.xDrive;
            drive.target = targetAngle;
            // Update stiffness and damping in case they have changed
            drive.stiffness = fingerStiffness;
            drive.damping = fingerDamping;
            body.xDrive = drive;
        }
    }
}
