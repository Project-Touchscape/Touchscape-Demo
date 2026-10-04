using UnityEngine;
using UnityEngine.XR.Hands;

public class HandSkeleton : MonoBehaviour
{
    private const int N_FINGERS = 5;
    private const int N_ACTIVE_BONES = 3;
    private XRHand hand;

    private GameObject         palmGameObject;
    private ArticulationBody   palmBody; 
    private ArticulationBody[] articulationBodies;
    private BoxCollider        palmCollider;
    private float boneWidth = 0.016f;
    private CapsuleCollider [] capsuleColliders;
    private int                lastFrameTeleport = 0;
    private bool               ghosted = false;
    private int                layerMask;

    public Transform handPalm;
    public SkinnedMeshRenderer handRenderer;

    [Range(0.1f, 10f)]
    public float strength = 1f;

    [SerializeField]
    [Tooltip("The mass of each finger bone; the palm will be 3x this.")]
    private float perBoneMass = 3.0f;

    [SerializeField]
    [Tooltip("The physics material that the hand uses.")]
    private PhysicsMaterial material = null;

    void Start()
    {
        // Fetch the XRHandSubsystem from the SubsystemManager
        var subsystems = new System.Collections.Generic.List<XRHandSubsystem>();
        SubsystemManager.GetSubsystems(subsystems);

        if (subsystems.Count > 0)
            hand = subsystems[0].rightHand;

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

        // Initialize the hand skeleton
        ConstructHand();
    }

    void Update()
    {
        if (hand == null)
            return;
    }

    public void ConstructHand() {
        ConstructPalm();

        capsuleColliders = new CapsuleCollider[N_FINGERS * N_ACTIVE_BONES];
        articulationBodies = new ArticulationBody[N_FINGERS * N_ACTIVE_BONES];

        // Create a small articulation chain for each finger using the tracked XR hand joints.
        for (int fingerIndex = 0; fingerIndex < N_FINGERS; fingerIndex++)
        {
            XRHandJointID[] jointIds = GetFingerJointIds((XRHandFingerID)fingerIndex);
            Transform parentTransform = palmGameObject.transform;

            for (int jointIndex = 0; jointIndex < N_ACTIVE_BONES; jointIndex++)
            {
                ConstructFingerJoint(fingerIndex, jointIndex, jointIds, ref parentTransform);
            }
        }
    }

    private bool TryGetJointPose(XRHandJointID jointId, out Pose pose)
    {
        pose = default;

        if (hand == null || !hand.isTracked)
            return false;

        XRHandJoint joint = hand.GetJoint(jointId);
        return joint.TryGetPose(out pose);
    }

    private void ConstructPalm()
    {
        // Create a new GameObject for the palm with an ArticulationBody and BoxCollider.
        palmGameObject = new GameObject(gameObject.name + " Palm", typeof(ArticulationBody), typeof(BoxCollider));
        palmGameObject.layer = gameObject.layer;
        palmBody = palmGameObject.GetComponent<ArticulationBody>();
        palmCollider = palmGameObject.GetComponent<BoxCollider>();

        // Use the XR Hand palm joint pose only when tracking is active.
        if (TryGetJointPose(XRHandJointID.Palm, out var palmPose))
        {
            palmGameObject.transform.SetPositionAndRotation(palmPose.position, palmPose.rotation);
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

    private void ConstructFingerJoint(int fingerIndex, int jointIndex, XRHandJointID[] jointIds, ref Transform parentTransform)
    {
        if (jointIds == null || jointIds.Length < 2)
            return;

        int currentJointIndex = jointIndex;
        int nextJointIndex = jointIndex + 1;

        if (currentJointIndex >= jointIds.Length || nextJointIndex >= jointIds.Length)
            return;

        // Gets current and next joint. We guard against tracking not being active or the joint data
        // not having been populated yet, which avoids the out-of-range array access seen at runtime.
        XRHandJoint currentJoint = hand.GetJoint(jointIds[currentJointIndex]);
        XRHandJoint nextJoint = hand.GetJoint(jointIds[nextJointIndex]);

        // Calculate the index for the articulation body and capsule collider arrays.
        int bodyIndex = fingerIndex * N_ACTIVE_BONES + jointIndex;

        // Create a new GameObject for the finger joint with a CapsuleCollider and ArticulationBody.
        GameObject capsuleGameObject = new GameObject(
            gameObject.name + " Finger " + fingerIndex + "-" + jointIndex,
            typeof(CapsuleCollider),
            typeof(ArticulationBody));
        capsuleGameObject.layer = gameObject.layer;
        capsuleGameObject.transform.SetParent(parentTransform, false);

        // Use the XR Hand joint pose only when valid tracking data exists.
        if (currentJoint.TryGetPose(out var currJointPose))
        {
            capsuleGameObject.transform.SetPositionAndRotation(currJointPose.position, currJointPose.rotation);
        }

        // Gets bone length from the distance between the current joint and the next joint.
        float boneLength = 0.02f;
        if (nextJoint.TryGetPose(out var nextJointPose) && currentJoint.TryGetPose(out var currentPose))
        {
            boneLength = Vector3.Distance(currentPose.position, nextJointPose.position);
        }

        // Creates capsule collider based on bone length and width
        CapsuleCollider capsule = capsuleGameObject.GetComponent<CapsuleCollider>();
        capsule.direction = 2;
        capsule.radius = boneWidth * 0.5f;
        capsule.height = boneLength + boneWidth;
        capsule.center = new Vector3(0f, 0f, 0f);
        capsule.material = material;
        capsuleColliders[bodyIndex] = capsule;

        // Creates articulation body for the finger joint with revolute joint type
        ArticulationBody body = capsuleGameObject.GetComponent<ArticulationBody>();
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
        articulationBodies[bodyIndex] = body;

        // Update the parent transform for the next joint in the finger chain to preserve hierarchy
        parentTransform = capsuleGameObject.transform;
    }
    private static XRHandJointID[] GetFingerJointIds(XRHandFingerID fingerId)
    {
        // XR Hand exposes each finger as a set of tracked joints. We map three representative joints
        // to the hand physics chain so the articulation body setup tracks the hand skeleton.
        switch (fingerId)
        {
            case XRHandFingerID.Thumb:
                return new[]
                {
                    XRHandJointID.ThumbMetacarpal,
                    XRHandJointID.ThumbProximal,
                    XRHandJointID.ThumbDistal,
                    XRHandJointID.ThumbTip
                };
            case XRHandFingerID.Index:
                return new[]
                {
                    XRHandJointID.IndexMetacarpal,
                    XRHandJointID.IndexProximal,
                    XRHandJointID.IndexDistal,
                    XRHandJointID.IndexTip
                };
            case XRHandFingerID.Middle:
                return new[]
                {
                    XRHandJointID.MiddleMetacarpal,
                    XRHandJointID.MiddleProximal,
                    XRHandJointID.MiddleDistal,
                    XRHandJointID.MiddleTip
                };
            case XRHandFingerID.Ring:
                return new[]
                {
                    XRHandJointID.RingMetacarpal,
                    XRHandJointID.RingProximal,
                    XRHandJointID.RingDistal,
                    XRHandJointID.RingTip
                };
            case XRHandFingerID.Little:
                return new[]
                {
                    XRHandJointID.LittleMetacarpal,
                    XRHandJointID.LittleProximal,
                    XRHandJointID.LittleDistal,
                    XRHandJointID.LittleTip
                };
            default:
                return new[] { XRHandJointID.Palm };
        }
    }

    public void UpdateHand()
    {
        // Safe access: XRHand joints are not guaranteed to be populated until tracking is active.
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
        }
    }
}
