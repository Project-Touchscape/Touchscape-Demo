using UnityEngine;
using UnityEngine.XR.Hands;

public class HandSkeleton : MonoBehaviour
{
    private const int N_FINGERS = 5;
    private const int N_ACTIVE_BONES = 3;
    private XRHand hand;
    private ArticulationBody   palmBody;
    private BoxCollider        palmCollider;
    private int                lastFrameTeleport = 0;
    private bool               ghosted = false;
    private int                layerMask;

    public GameObject palm;
    public GameObject[] initialFingerJoints = new GameObject[N_FINGERS];

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

    void Start()
    {
        // Build a collision mask for the hand so it can interact with the scene without
        // colliding with layers that should be ignored.
        int myLayer = gameObject.layer;
        layerMask = 0;
        for (int i = 0; i < 32; i++)
        {
            if (!Physics.GetIgnoreLayerCollision(myLayer, i))
            {
                layerMask |= 1 << i;
            }
        }

        // Initialize palm articulation body and collider
        ConstructPalm();
        ConstructFingers();
    }

    void Update()
    {
        if (hand == null)
            return;
    }

    private void ConstructPalm()
    {
        // Adds articulation body to current body to preserve hierarchy
        palmBody = gameObject.GetComponent<ArticulationBody>();
        if (palmBody == null)
        {
            palmBody = gameObject.AddComponent<ArticulationBody>();
        }

        // Adds collider to palm object
        palmCollider = palm.GetComponent<BoxCollider>();
        if (palmCollider == null)
        {
            palmCollider = palm.AddComponent<BoxCollider>();
        }

        // Palm properties: mass, collider size, and physics material
        palmCollider.center = new Vector3(0f, 0.005f, -0.015f);
        palmCollider.size = new Vector3(0.06f, 0.02f, 0.07f);
        palmCollider.material = material;
        
        palmBody.mass = perBoneMass * 3f;
        palmBody.immovable = false;
        palmBody.solverIterations = 60;
        palmBody.solverVelocityIterations = 20;
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
        ArticulationBody body = currentJoint.GetComponent<ArticulationBody>();
        if (body == null)
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
    }

    public void UpdateHand()
    {
        /*// Safe access: XRHand joints are not guaranteed to be populated until tracking is active.
        // Calling GetJoint() before that point can throw because the internal joint array is still empty.
        if (hand == null || palmBody == null || !hand.isTracked)
            return;

        if (!TryGetJointPose(XRHandJointID.Palm, out var palmPose))
            return;

        // Keep the physics palm aligned to the tracked XR palm.
        palmBody.TeleportRoot(palmPose.position, palmPose.rotation);

        if (handPalm != null)
        {
            handPalm.position = palmBody.transform.position - (palmBody.transform.forward * 0.06f);
            handPalm.rotation = palmBody.transform.rotation * Quaternion.Euler(
                hand.handedness == Handedness.Left ? 180f : 0f,
                hand.handedness == Handedness.Left ? 90f : -90f,
                0f);
        }

        // Update the finger articulation bodies from valid tracked joint poses only.
        for (int fingerIndex = 0; fingerIndex < N_FINGERS; fingerIndex++)
        {
            XRHandJointID[] jointIds = GetFingerJointIds((XRHandFingerID)fingerIndex);
            if (jointIds == null || jointIds.Length == 0)
                continue;

            for (int jointIndex = 0; jointIndex < N_ACTIVE_BONES; jointIndex++)
            {
                int bodyIndex = fingerIndex * N_ACTIVE_BONES + jointIndex;
                if (bodyIndex >= articulationBodies.Length || articulationBodies[bodyIndex] == null)
                    continue;

                if (jointIndex >= jointIds.Length)
                    continue;

                if (!TryGetJointPose(jointIds[jointIndex], out var fingerPose))
                    continue;

                ArticulationBody body = articulationBodies[bodyIndex];
                body.transform.SetPositionAndRotation(fingerPose.position, fingerPose.rotation);
            }
        }*/
    }
}
