using UnityEngine;
using UnityEngine.XR.Hands;

public class HandSkeleton : MonoBehaviour
{
    private const int N_FINGERS = 5;
    private XRHand hand;
    private ArticulationBody   palmBody;
    private BoxCollider        palmCollider;
    private int                lastFrameTeleport = 0;
    private bool               ghosted = false;
    private int                layerMask;

    public GameObject palm;
    public GameObject[] fingers = new GameObject[N_FINGERS];

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
    }

    void Update()
    {
        if (hand == null)
            return;
    }

    private void ConstructPalm()
    {
        // Add collider/articulation body components to palm game object
        palmBody = palm.GetComponent<ArticulationBody>();
        if (palmBody == null)
        {
            palmBody = palm.AddComponent<ArticulationBody>();
        }

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
        palmBody.immovable = true;
        palmBody.solverIterations = 60;
        palmBody.solverVelocityIterations = 20;
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
