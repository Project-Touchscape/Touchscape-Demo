using UnityEngine;
using UnityEngine.XR.Hands;
using System.Collections.Generic;
using System.Text;
using Unity.VisualScripting;

public class HandSkeleton : MonoBehaviour
{
    public enum Handedness
    {
        Left,
        Right
    }

    private const int N_FINGERS = 5;
    private const int N_ACTIVE_BONES = 4;
    private const string FingerColliderName = "Finger Collider";

    public Handedness handedness = Handedness.Right;
    public HapticRenderClient haptics;
    public GameObject palm;
    public Transform trackedRoot;
    public GameObject[] fingers = new GameObject[N_FINGERS];
    public Transform[] trackedFingers = new Transform[N_FINGERS];
    public float[] fingerBoneWidths = new float[N_ACTIVE_BONES] {0.020f, 0.020f, 0.018f, 0.016f};
    public float[] thumbBoneWidths = new float[N_ACTIVE_BONES-1] {0.025f, 0.023f, 0.021f};

    public float[] fingerJointMinimums = new float[N_ACTIVE_BONES] {-10f, -10f, -10f, -10f};
    public float[] fingerJointMaximums = new float[N_ACTIVE_BONES] {10f, 89f, 89f, 89f};

    public float[] thumbJointMinimums = new float[N_ACTIVE_BONES-1] {-45f, -45f, -45f};
    public float[] thumbJointMaximums = new float[N_ACTIVE_BONES-1] {45f, 45f, 45f};

    public float fingerSplayMinimum = -10f;
    public float fingerSplayMaximum = 10f;

    public float thumbSplayMinimum = -45f;
    public float thumbSplayMaximum = 45f;

    [SerializeField]
    [Tooltip("The mass of each finger bone")]
    public float perBoneMass = 0.006f;
    public float palmMass = 0.3f;

    [SerializeField]
    [Tooltip("The physics material that the hand uses.")]
    public PhysicsMaterial material = null;

    public bool useGravity = false;

    [Range(0f, 1f)]
    public float palmPosStiffness = 0.9f;

    public float palmPosDamping = 1f;
    [Range(0f, 1f)]
    public float palmRotStiffness = 0.9f;

    public float palmRotDamping = 1f;

    public float palmMaxForce = 20f;
    public float palmMaxTorque = 40f;

    [Range(0f, 1f)]
    public float fingerStiffness = 1f;

    public float fingerDamping = 1f;

    public float fingerMaxTorque = 40f;

    [Header("Articulation Chain Rebuild")]
    [Min(0.0001f)] public float rootPositionRebuildThreshold = 0.002f;
    [Min(0.0001f)] public float boneLengthRebuildThreshold = 0.002f;

    /*[Header("Joint Debug")]
    public bool logSplayJoint = false;
    [Min(0)]
    public int debugFingerIndex = 0;
    [Min(0)]
    public int debugJointIndex = 0;
    [Min(0.02f)]
    public float jointLogInterval = 0.5f;
    private float nextJointLogTime;*/

    [Header("Joint Drive Diagnostics")]
    public bool logJointDriveDiagnostics = true;
    [Min(0.05f)] public float jointDriveDiagnosticInterval = 0.5f;
    [Min(0)] public int diagnosticFingerIndex = 0;
    [Min(0)] public int diagnosticJointIndex = 1;
    private float nextJointDriveDiagnosticTime;
    public bool logArticulationStateDiagnostics = false;
    [Min(0.05f)] public float articulationStateDiagnosticInterval = 0.5f;
    private float nextArticulationStateDiagnosticTime;

    private IntegratedHapticNode rootNode;

    private Vector3 palmColliderOffset = new Vector3(0.0047f, -0.0043f, 0.0091f);
    private Vector3 palmColliderRot = new Vector3(5.31f, 4.947f, -0.786f);
    private Vector3 palmColliderScale = new Vector3(0.07241f, 0.03259f, 0.07990f);

    private List<ArticulationBody> articulationBodies = new List<ArticulationBody>();

    // First joint has splay, second and third do not have splay
    private int[] perJointDOF = new int[N_ACTIVE_BONES] {1, 2, 1, 1};

    private HapticNode[] fingertipNodes = new HapticNode[N_FINGERS];
    private Vector3[] prevFingertipLinearMomenta = new Vector3[N_FINGERS];
    private Vector3[] fingertipLinearMomenta = new Vector3[N_FINGERS];
    private Vector3[] prevFingertipForces = new Vector3[N_FINGERS];
    private Vector3[] fingertipForces = new Vector3[N_FINGERS];
    private Vector3[] prevFingertipInertialForces = new Vector3[N_FINGERS];
    private Vector3[] fingertipInertialForces = new Vector3[N_FINGERS];

    private bool prevUseGravity = false;
    private Vector3[] builtFingerRootPositions = new Vector3[N_FINGERS];
    private float[,] builtBoneLengths = new float[N_FINGERS, N_ACTIVE_BONES];
    private float[,,] initialJointOffsets =
        new float[N_FINGERS, N_ACTIVE_BONES, 2];
    private bool hasBuiltGeometry;
    private bool hasInitialJointOffsets;
    private bool[,,,] angleLimitWarnings =
        new bool[N_FINGERS, N_ACTIVE_BONES, 2, 2];

    void Start()
    {
        // Initialize palm articulation body and collider
        ConstructPalm();
        ConstructFingers();
        CaptureFingerState();
    }

    void Update()
    {

    }

    void FixedUpdate()
    {
        if (HasFingerGeometryChanged()) {
            RebuildFingers();

            // Logs warning
            Debug.LogWarning("Tracked geometry has changed. Rebuilding articulation chain.");
        }

        // Checks for change in gravity state
        if (prevUseGravity != useGravity)
        {
            prevUseGravity = useGravity;
            // Update the gravity state of all articulation bodies
            foreach (ArticulationBody ab in articulationBodies)
            {
                ab.useGravity = useGravity;
            }
        }

        // Update physics for root haptic node
        if (rootNode != null)
        {
            // Update parameters
            rootNode.posStiffness = palmPosStiffness;
            rootNode.posDamping = palmPosDamping;
            rootNode.rotStiffness = palmRotStiffness;
            rootNode.rotDamping = palmRotDamping;
            rootNode.maxPhysicsForce = palmMaxForce;
            rootNode.maxPhysicsTorque = palmMaxTorque;

            // Physics timestep
            rootNode.UpdatePhysics();
        }

        // Update the articulation bodies' target angles based on the tracked joint rotations
        if (fingers == null || trackedFingers == null)
            return;

        for (int fingerIndex = 0;
             fingerIndex < Mathf.Min(
                 N_FINGERS,
                 Mathf.Min(fingers.Length, trackedFingers.Length));
             fingerIndex++)
        {
            GameObject currentJoint = fingers[fingerIndex];

            // Get the corresponding tracked joint transform
            Transform trackedJoint = trackedFingers[fingerIndex];

            if (trackedJoint == null || currentJoint == null)
                continue;

            for (int jointIndex = 0;
                 jointIndex < GetBonesPerFinger(fingerIndex);
                 jointIndex++)
            {
                // Update the articulation body's target angle based on the tracked joint's rotation
                UpdateJointParams(
                    fingerIndex,
                    jointIndex,
                    currentJoint,
                    trackedJoint);

                // Move to the next joint in the finger hierarchy.
                Transform nextPhysicsJoint = GetBoneChild(currentJoint.transform);
                Transform nextTrackedJoint = GetBoneChild(trackedJoint);
                if (nextPhysicsJoint == null || nextTrackedJoint == null)
                    break;

                currentJoint = nextPhysicsJoint.gameObject;
                trackedJoint = nextTrackedJoint;
            }

            // Updates fingertip haptic node
            UpdateFingertipNode(fingerIndex);
        }

        if (logArticulationStateDiagnostics &&
            Time.time >= nextArticulationStateDiagnosticTime)
        {
            nextArticulationStateDiagnosticTime =
                Time.time + articulationStateDiagnosticInterval;
            LogArticulationStateDiagnostics();
        }
    }

    private bool IsThumb(int fingerIndex)
    {
        return fingerIndex == N_FINGERS - 1;
    }

    private int GetBonesPerFinger(int fingerIndex)
    {
        return IsThumb(fingerIndex) ? N_ACTIVE_BONES-1 : N_ACTIVE_BONES;
    }

    private int GetJointDOF(int fingerIndex, int jointIndex)
    {
        return IsThumb(fingerIndex) ? perJointDOF[jointIndex + 1] : perJointDOF[jointIndex];
    }

    private bool HasFingerGeometryChanged()
    {
        if (trackedRoot == null || fingers == null || trackedFingers == null)
            return false;

        if (!hasBuiltGeometry)
            return true;

        for (int fingerIndex = 0; fingerIndex < N_FINGERS; fingerIndex++)
        {
            Transform trackedFinger = trackedFingers[fingerIndex];
            if (trackedFinger == null)
                continue;

            Vector3 rootPosition = trackedRoot.InverseTransformPoint(trackedFinger.position);
            if (Vector3.Distance(rootPosition, builtFingerRootPositions[fingerIndex]) > rootPositionRebuildThreshold)
            {
                return true;
            }

            Transform trackedJoint = trackedFinger;
            for (int jointIndex = 0; jointIndex < GetBonesPerFinger(fingerIndex); jointIndex++)
            {
                if (trackedJoint == null)
                    break;

                float length = GetBoneVector(trackedJoint).magnitude;
                if (Mathf.Abs(length - builtBoneLengths[fingerIndex, jointIndex]) > boneLengthRebuildThreshold)
                {
                    return true;
                }

                trackedJoint = GetBoneChild(trackedJoint);
                if (trackedJoint == null)
                    break;
            }
        }

        return false;
    }

    private void RebuildFingers()
    {
        if (fingers == null || trackedFingers == null)
            return;

        foreach (ArticulationBody body in articulationBodies)
            body.enabled = false;

        int fingerCount = Mathf.Min(
            N_FINGERS,
            Mathf.Min(fingers.Length, trackedFingers.Length));
        for (int fingerIndex = 0; fingerIndex < fingerCount; fingerIndex++)
        {
            GameObject currentJoint = fingers[fingerIndex];
            Transform trackedJoint = trackedFingers[fingerIndex];
            if (currentJoint == null || trackedJoint == null)
                continue;

            if (trackedRoot != null)
            {
                Vector3 relativeRootPosition =
                    trackedRoot.InverseTransformPoint(trackedJoint.position);
                Quaternion relativeRootRotation =
                    Quaternion.Inverse(trackedRoot.rotation) *
                    trackedJoint.rotation;

                currentJoint.transform.position =
                    gameObject.transform.TransformPoint(relativeRootPosition);
                currentJoint.transform.rotation =
                    gameObject.transform.rotation * relativeRootRotation;
            }

            for (int jointIndex = 0; jointIndex < GetBonesPerFinger(fingerIndex); jointIndex++)
            {
                Transform physicsChild = GetBoneChild(currentJoint.transform);
                Vector3 trackedBone = GetBoneVector(trackedJoint);
                if (physicsChild == null || trackedBone.sqrMagnitude <= Mathf.Epsilon)
                    break;

                Vector3 localDirection = physicsChild.localPosition;
                localDirection = localDirection.sqrMagnitude <= Mathf.Epsilon
                    ? Vector3.forward
                    : localDirection.normalized;
                physicsChild.localPosition =
                    localDirection * trackedBone.magnitude;

                Transform colliderTransform =
                    currentJoint.transform.Find(FingerColliderName);
                CapsuleCollider collider = colliderTransform == null
                    ? null
                    : colliderTransform.GetComponent<CapsuleCollider>();
                if (collider != null)
                {
                    UpdateBoneCollider(
                        fingerIndex,
                        jointIndex,
                        collider,
                        trackedBone.magnitude);
                }

                trackedJoint = GetBoneChild(trackedJoint);
                currentJoint = physicsChild.gameObject;
                if (trackedJoint == null)
                    break;
            }
        }

        foreach (ArticulationBody body in articulationBodies)
            body.enabled = true;

        CaptureFingerState();
    }

    private void CaptureFingerState()
    {
        if (fingers == null || trackedFingers == null)
            return;

        int fingerCount = Mathf.Min(
            N_FINGERS,
            Mathf.Min(fingers.Length, trackedFingers.Length));
        for (int fingerIndex = 0; fingerIndex < fingerCount; fingerIndex++)
        {
            Transform trackedJoint = trackedFingers[fingerIndex];
            GameObject currentJoint = fingers[fingerIndex];
            for (int jointIndex = 0;
                 jointIndex < GetBonesPerFinger(fingerIndex) &&
                 trackedJoint != null &&
                 currentJoint != null;
                 jointIndex++)
            {
                if (jointIndex == 0 && trackedRoot != null)
                {
                    builtFingerRootPositions[fingerIndex] =
                        trackedRoot.InverseTransformPoint(
                            trackedJoint.position);
                }

                builtBoneLengths[fingerIndex, jointIndex] =
                    GetBoneVector(trackedJoint).magnitude;

                ArticulationBody body =
                    currentJoint.GetComponent<ArticulationBody>();
                Transform physicsJoint = currentJoint.transform;
                Transform parentJoint = physicsJoint.parent;
                if (parentJoint != null && body != null)
                {
                    Vector3 incomingBone =
                        physicsJoint.position - parentJoint.position;
                    Vector3 outgoingBone = GetBoneVector(physicsJoint);

                    float physicsFlexion = GetRelativeDriveAngle(
                            physicsJoint,
                            incomingBone,
                            outgoingBone,
                            0);
                    float physicsSplay = GetRelativeDriveAngle(
                            physicsJoint,
                            incomingBone,
                            outgoingBone,
                            1);

                    initialJointOffsets[fingerIndex, jointIndex, 0] =
                        Mathf.DeltaAngle(
                            GetJointPositionDegrees(body, 0),
                            physicsFlexion);
                    initialJointOffsets[fingerIndex, jointIndex, 1] =
                        Mathf.DeltaAngle(
                            GetJointPositionDegrees(body, 1),
                            physicsSplay);

                    ArticulationDrive xDrive = body.xDrive;
                    float xOffset =
                        initialJointOffsets[fingerIndex, jointIndex, 0];
                    xDrive.lowerLimit =
                        GetFlexionMinimum(fingerIndex, jointIndex) - xOffset;
                    xDrive.upperLimit =
                        GetFlexionMaximum(fingerIndex, jointIndex) - xOffset;
                    body.xDrive = xDrive;

                    if (body.jointPosition.dofCount > 1)
                    {
                        ArticulationDrive yDrive = body.yDrive;
                        float yOffset =
                            initialJointOffsets[fingerIndex, jointIndex, 1];
                        yDrive.lowerLimit =
                            GetSplayMinimum(fingerIndex) - yOffset;
                        yDrive.upperLimit =
                            GetSplayMaximum(fingerIndex) - yOffset;
                        body.yDrive = yDrive;
                    }
                }

                trackedJoint = GetBoneChild(trackedJoint);
                Transform physicsChild = GetBoneChild(currentJoint.transform);
                currentJoint = physicsChild == null
                    ? null
                    : physicsChild.gameObject;
            }
        }

        hasBuiltGeometry = true;
        hasInitialJointOffsets = true;
    }

    private float GetJointPositionDegrees(
        ArticulationBody body,
        int axisType)
    {
        return body.jointPosition.dofCount > axisType
            ? body.jointPosition[axisType] * Mathf.Rad2Deg
            : 0f;
    }

    private float GetFlexionMinimum(int fingerIndex, int jointIndex)
    {
        return IsThumb(fingerIndex)
            ? thumbJointMinimums[jointIndex]
            : fingerJointMinimums[jointIndex];
    }

    private float GetFlexionMaximum(int fingerIndex, int jointIndex)
    {
        return IsThumb(fingerIndex)
            ? thumbJointMaximums[jointIndex]
            : fingerJointMaximums[jointIndex];
    }

    private float GetSplayMinimum(int fingerIndex)
    {
        return IsThumb(fingerIndex)
            ? thumbSplayMinimum
            : fingerSplayMinimum;
    }

    private float GetSplayMaximum(int fingerIndex)
    {
        return IsThumb(fingerIndex)
            ? thumbSplayMaximum
            : fingerSplayMaximum;
    }

    private void ConstructPalm()
    {
        // Adds root articulation body to current body to preserve hierarchy
        ArticulationBody rootBody = gameObject.GetComponent<ArticulationBody>();
        if (rootBody == null)
        {
            rootBody = gameObject.AddComponent<ArticulationBody>();
        }

        rootBody.mass = Mathf.Epsilon;
        rootBody.immovable = false;
        rootBody.useGravity = useGravity;
        rootBody.linearDamping = 0f;
        rootBody.angularDamping = 0f;
        rootBody.solverIterations = 60;
        rootBody.solverVelocityIterations = 20;
        articulationBodies.Add(rootBody);

        // Constructs haptic node at root
        if (!gameObject.TryGetComponent<IntegratedHapticNode>(out rootNode))
            rootNode = gameObject.AddComponent<IntegratedHapticNode>();

        rootNode.haptics = haptics;
        rootNode.trackedTransform = trackedRoot;
        rootNode.useChainedMass = true;
        rootNode.posStiffness = palmPosStiffness;
        rootNode.posDamping = palmPosDamping;
        rootNode.rotStiffness = palmRotStiffness;
        rootNode.rotDamping = palmRotDamping;

        // Normalizes max force by total mass
        rootNode.maxPhysicsForce = palmMaxForce;
        rootNode.maxPhysicsTorque = palmMaxTorque;

        // The collider must be added to a child object to allow rotation.
        Transform existingPalmCollider =
            palm.transform.Find(palm.name + " Collider");

        GameObject palmChild = existingPalmCollider == null
            ? new GameObject(palm.name + " Collider")
            : existingPalmCollider.gameObject;

        palmChild.transform.SetParent(palm.transform, false);
        palmChild.layer = palm.layer;

        BoxCollider palmCollider = palmChild.GetComponent<BoxCollider>();
        if (palmCollider == null)
        {
            palmCollider = palmChild.AddComponent<BoxCollider>();
        }

        // Apply the same local palm offset to existing and newly created colliders.
        // Only the lateral component is mirrored; palm depth and height remain
        // consistent between the left and right hands.
        float lateralSign = handedness == Handedness.Left ? -1f : 1f;

        palmChild.transform.localPosition = new Vector3(
            lateralSign * palmColliderOffset.x,
            palmColliderOffset.y,
            palmColliderOffset.z);

        Vector3 mirroredPalmColliderRot = handedness == Handedness.Left
            ? new Vector3(palmColliderRot.x, -palmColliderRot.y, -palmColliderRot.z)
            : palmColliderRot;

        palmChild.transform.localRotation = Quaternion.Euler(mirroredPalmColliderRot);
        palmChild.transform.localScale = palmColliderScale;

        palmCollider.center = Vector3.zero;
        palmCollider.size = Vector3.one;

        // Always apply the configured hand material, including when the collider
        // was already present on the child.
        palmCollider.material = material;

        // Adds fixed articulation body to palm child object
        ArticulationBody palmBody = palmChild.GetComponent<ArticulationBody>();
        if (palmBody == null)
        {
            palmBody = palmChild.AddComponent<ArticulationBody>();
        }

        palmBody.mass = palmMass;
        palmBody.useGravity = useGravity;
        palmBody.linearDamping = 0f;
        palmBody.angularDamping = 0f;
        palmBody.solverIterations = 60;
        palmBody.solverVelocityIterations = 20;
        palmBody.jointType = ArticulationJointType.FixedJoint;
        palmBody.collisionDetectionMode = CollisionDetectionMode.Continuous;
        articulationBodies.Add(palmBody);
    }

    private void ConstructFingers()
    {
        // Each entry is the first joint GameObject for one finger. The remaining joints
        // are found by walking the existing GameObject hierarchy.
        if (fingers == null)
            return;

        for (int fingerIndex = 0; fingerIndex < N_FINGERS; fingerIndex++)
        {
            GameObject currentJoint = fingers[fingerIndex];

            for (int jointIndex = 0;
                 jointIndex < GetBonesPerFinger(fingerIndex);
                 jointIndex++)
            {
                if (currentJoint == null)
                    break;

                ConstructFingerJoint(fingerIndex, jointIndex, currentJoint);
                if (GetBoneChild(currentJoint.transform) == null)
                    break;

                currentJoint = GetBoneChild(currentJoint.transform).gameObject;
            }

            // Constructs haptic node for the fingertip
            if (currentJoint != null)
                ConstructFingertipNode(fingerIndex, currentJoint);
        }
    }

    private void ConstructFingerJoint(int fingerIndex, int jointIndex, GameObject currentJoint)
    {
        bool isThumb = IsThumb(fingerIndex);
        int dof = GetJointDOF(fingerIndex, jointIndex);

        Transform jointTransform = currentJoint.transform;

        Vector3 boneDirection = GetBoneVector(jointTransform);

        // Configure the collider on a separate child.
        Transform colliderTransform = jointTransform.Find(FingerColliderName);

        if (colliderTransform == null)
        {
            GameObject colliderObject = new GameObject(FingerColliderName);
            colliderTransform = colliderObject.transform;
            colliderTransform.SetParent(jointTransform, false);
        }

        colliderTransform.gameObject.layer = currentJoint.layer;
        colliderTransform.localPosition = Vector3.zero;

        if (boneDirection.sqrMagnitude > Mathf.Epsilon)
        {
            colliderTransform.rotation = Quaternion.LookRotation(
                boneDirection.normalized,
                jointTransform.up
            );
        }

        CapsuleCollider capsule = colliderTransform.GetComponent<CapsuleCollider>();

        if (capsule == null)
            capsule = colliderTransform.gameObject.AddComponent<CapsuleCollider>();

        UpdateBoneCollider(fingerIndex, jointIndex, capsule, boneDirection.magnitude);

        // Create the articulation body.
        if (!currentJoint.TryGetComponent<ArticulationBody>(out var body))
            body = currentJoint.AddComponent<ArticulationBody>();

        body.mass = perBoneMass;
        body.useGravity = useGravity;
        body.linearDamping = 0f;
        body.angularDamping = 0f;
        body.jointFriction = 0f;
        body.collisionDetectionMode = CollisionDetectionMode.Continuous;
        body.solverIterations = 60;
        body.solverVelocityIterations = 20;

        // Let Unity derive the connected anchor frames from the existing
        // hierarchy. The initial articulation pose is then the hierarchy pose,
        // rather than a manually reconstructed tracked-pose frame.
        body.anchorPosition = Vector3.zero;
        body.anchorRotation = Quaternion.identity;

        if (dof > 0)
        {
            body.jointType = dof == 1
                ? ArticulationJointType.RevoluteJoint
                : ArticulationJointType.SphericalJoint;

            body.twistLock = ArticulationDofLock.LimitedMotion;

            float lowerLimit = isThumb
                ? thumbJointMinimums[jointIndex]
                : fingerJointMinimums[jointIndex];

            float upperLimit = isThumb
                ? thumbJointMaximums[jointIndex]
                : fingerJointMaximums[jointIndex];

            body.xDrive = new ArticulationDrive
            {
                driveType = ArticulationDriveType.Force,
                target = 0f,
                stiffness = 0f,
                damping = 0f,
                forceLimit = 0f,
                lowerLimit = lowerLimit,
                upperLimit = upperLimit
            };

            if (dof > 1)
            {
                body.swingYLock = ArticulationDofLock.LimitedMotion;
                body.swingZLock = ArticulationDofLock.LockedMotion;

                float splayLowerLimit = isThumb
                    ? thumbSplayMinimum
                    : fingerSplayMinimum;

                float splayUpperLimit = isThumb
                    ? thumbSplayMaximum
                    : fingerSplayMaximum;

                body.yDrive = new ArticulationDrive
                {
                    driveType = ArticulationDriveType.Force,
                    target = 0f,
                    stiffness = 0f,
                    damping = 0f,
                    forceLimit = 0f,
                    lowerLimit = splayLowerLimit,
                    upperLimit = splayUpperLimit
                };
            }
        }
        else
        {
            body.jointType = ArticulationJointType.FixedJoint;
        }

        articulationBodies.Add(body);
    }

    private void ConstructFingertipNode(int fingerIndex, GameObject currentJoint)
    {
        if (currentJoint == null || trackedFingers[fingerIndex] == null)
            return;

        if (!currentJoint.TryGetComponent<HapticNode>(out HapticNode node))
            node = currentJoint.AddComponent<HapticNode>();

        // Gets tracked joint
        Transform trackedJoint = trackedFingers[fingerIndex];

        for (int i = 0; i < GetBonesPerFinger(fingerIndex); i++)
        {
            trackedJoint = GetBoneChild(trackedJoint);
            if (trackedJoint == null)
                break;
        }

        if (trackedJoint == null)
            return;

        node.haptics = haptics;
        node.trackedTransform = trackedJoint;
        fingertipNodes[fingerIndex] = node;
    }

    /// <summary>
    /// Gets the direction of the bone from the current joint to the next joint.
    /// </summary>
    /// <param name="currentJoint">The current joint.</param>
    /// <returns>The direction of the bone.</returns>
    private Vector3 GetBoneVector(Transform currentJoint)
    {
        Transform nextJoint = GetBoneChild(currentJoint);

        if (nextJoint == null)
            return currentJoint.forward;

        return nextJoint.position - currentJoint.position;
    }

    private Transform GetBoneChild(Transform joint)
    {
        if (joint == null)
            return null;

        for (int childIndex = 0; childIndex < joint.childCount; childIndex++)
        {
            Transform child = joint.GetChild(childIndex);
            if (child.name != FingerColliderName)
                return child;
        }

        return null;
    }

    private void UpdateBoneCollider(int fingerIndex, int jointIndex, CapsuleCollider collider, float boneLength)
    {
        // Gets the bone width for the current joint
        float boneWidth = IsThumb(fingerIndex) ? thumbBoneWidths[jointIndex] : fingerBoneWidths[jointIndex];

        // Shortens the bone length for the last joint to be tangent to the fingertip
        if (jointIndex == GetBonesPerFinger(fingerIndex) - 1)
            boneLength -= boneWidth * 0.5f;

        collider.direction = 2;
        collider.radius = boneWidth * 0.5f;
        collider.height = boneLength + boneWidth;
        collider.center = new Vector3(0f, 0f, boneLength * 0.5f);
        collider.material = material;
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
    private void UpdateJointParams(
        int fingerIndex,
        int jointIndex,
        GameObject currentJoint,
        Transform trackedJoint)
    {
        if (currentJoint == null || trackedJoint == null)
            return;

        float dof = GetJointDOF(fingerIndex, jointIndex);
        ArticulationBody body = currentJoint.GetComponent<ArticulationBody>();

        if (body == null)
        {
            return;
        }

        if (dof > 0)
        {
            // Measure the tracked bones around the physics joint's local X axis. The
            // articulation drive rotates around this axis, and using trackedJoint.right
            // here can introduce a thumb flexion offset when the two rigs use different
            // local axes.

            body.xDrive = UpdateAxisDrive(
                body,
                body.xDrive,
                trackedJoint,
                trackedJoint.parent,
                fingerIndex,
                jointIndex,
                0);

            if (dof > 1)
            {
                body.yDrive = UpdateAxisDrive(
                    body,
                    body.yDrive,
                    trackedJoint,
                    trackedJoint.parent,
                    fingerIndex,
                    jointIndex,
                    1);

                /*if (logSplayJoint &&
                    fingerIndex == debugFingerIndex &&
                    jointIndex == debugJointIndex &&
                    Time.time >= nextJointLogTime)
                {
                    nextJointLogTime = Time.time + jointLogInterval;
                    LogSplayJoint(
                        fingerIndex,
                        jointIndex,
                        currentJoint,
                        trackedJoint,
                        body,
                        parentBoneDirection,
                        trackedBoneDirection,
                        rightAxis,
                        upAxis);
                }*/
            }
        }
    }

    private ArticulationDrive UpdateAxisDrive(
        ArticulationBody body,
        ArticulationDrive drive,
        Transform trackedJoint,
        Transform parentJoint,
        int fingerIndex,
        int jointIndex,
        int axisNum)
    {
        if (parentJoint == null)
            return drive;

        Vector3 incomingBone = trackedJoint.position - parentJoint.position;
        Vector3 outgoingBone = GetBoneVector(trackedJoint);

        Vector3 axis = axisNum == 0
            ? Utils.GetArticulationRightAxis(body)
            : Utils.GetArticulationUpAxis(body);
        float trackedAngle = GetRelativeDriveAngle(
            trackedJoint,
            incomingBone,
            outgoingBone,
            axisNum);

        float targetAngle = Mathf.DeltaAngle(
            hasInitialJointOffsets
                ? initialJointOffsets[fingerIndex, jointIndex, axisNum]
                : 0f,
            trackedAngle);

        WarnIfAngleLimitReached(
            body,
            targetAngle,
            trackedAngle,
            fingerIndex,
            jointIndex,
            axisNum);

        drive.target = targetAngle;

        if (logJointDriveDiagnostics &&
            fingerIndex == diagnosticFingerIndex &&
            axisNum == 0 &&
            jointIndex == diagnosticJointIndex &&
            Time.time >= nextJointDriveDiagnosticTime)
        {
            nextJointDriveDiagnosticTime =
                Time.time + jointDriveDiagnosticInterval;
            LogJointDriveDiagnostic(
                body,
                trackedJoint,
                parentJoint,
                incomingBone,
                outgoingBone,
                axis,
                trackedAngle,
                targetAngle,
                fingerIndex,
                jointIndex);
        }

        // Update stiffness and damping in case they have changed

        // Finds moment of inertia of current body and its children
        Utils.GetChainedMassProperties(body, out float totalMass, out Vector3 com, out Matrix4x4 totalInertia);

        // Applies coefficients to get necessary angular acceleration
        Vector3 jointPoint = body.transform.position;

        float axisInertia = Utils.InertiaAboutAxisAtPoint(
            totalInertia,
            totalMass,
            com,
            jointPoint,
            axis);

        drive.stiffness = fingerStiffness * axisInertia * Utils.GetMaxStiffness();
        drive.damping = fingerDamping * axisInertia * Utils.GetMaxDamping();
        drive.forceLimit = fingerMaxTorque;

        float currentPhysicsAngle = body.jointPosition[axisNum] * Mathf.Rad2Deg;
        float angularError = (currentPhysicsAngle - targetAngle) * Mathf.Deg2Rad;

        float driveTorque = -(drive.stiffness * angularError + drive.damping * body.jointVelocity[axisNum]);

        // Clamp the drive torque to the torque limit
        driveTorque = Mathf.Clamp(driveTorque, -drive.forceLimit, drive.forceLimit);

        HapticNode fingertipNode = fingertipNodes[fingerIndex];
        if (fingertipNode == null)
            return drive;

        Vector3 jointToFingertip =
            fingertipNode.trackedTransform.position - trackedJoint.position;

        // Gets relationship between joint torque and fingertip force
        Vector3 jacobianColumn = Vector3.Cross(axis, jointToFingertip);
        float jacobianMagnitudeSquared = jacobianColumn.sqrMagnitude;
        if (jacobianMagnitudeSquared <= Mathf.Epsilon)
            return drive;

        jacobianColumn /= jacobianMagnitudeSquared;

        // Gets force at fingertip
        fingertipForces[fingerIndex] += jacobianColumn * driveTorque;

        // Convert this joint's angular velocity into its angular momentum
        // contribution, then map that contribution to fingertip linear
        // momentum using the joint-to-fingertip lever arm.
        float angularMomentum = axisInertia *
            body.jointVelocity[axisNum];

        Vector3 linearMomentumContribution = jacobianColumn * angularMomentum;
        fingertipLinearMomenta[fingerIndex] += linearMomentumContribution;

        // Gets inertial forces at fingertips due to root node acceleration
        float effectiveJointTorque = 0f;
        // Gets all articulation bodies upstream of joint
        ArticulationBody[] articulationBodies = gameObject.GetComponentsInChildren<ArticulationBody>();
        foreach (ArticulationBody ab in articulationBodies) {
            Utils.GetArticulationBodyWrench(
                    rootNode.GetPhysicsAb(),
                    rootNode.GetPhysicsCOM(),
                    ab,
                    -rootNode.GetPhysicsAccel(),
                    -rootNode.GetPhysicsAngAccel(),
                    out Vector3 childForce,
                    out Vector3 childTorque);

            // Finds torque on the joint due to the linear acceleration and angular acceleration
            Vector3 jointToCOM = ab.worldCenterOfMass - jointPoint;
            Vector3 effectiveTorque = Vector3.Cross(jointToCOM, childForce) + childTorque;
            effectiveJointTorque += Vector3.Dot(effectiveTorque, axis);
        }

        // Finds the contribution of the effective joint torque to the fingertip inertial force
        fingertipInertialForces[fingerIndex] += jacobianColumn * effectiveJointTorque;

        /*Debug.Log(
            $"fingerIndex: {fingerIndex}\n" +
            $"jointToFingertip: {jointToFingertip}\n" +
            $"targetAngle: {targetAngle:F3}, currentPhysicsAngle: {currentPhysicsAngle:F3}, driveTorque: {driveTorque:F3}\n" +
            $"fingertipForce: {fingertipForces[fingerIndex]}\n" +
            $"fingertipLinearMomentum: {fingertipLinearMomenta[fingerIndex]}"
            );*/

        return drive;
    }

    private void WarnIfAngleLimitReached(
        ArticulationBody body,
        float targetAngle,
        float relativeAngle,
        int fingerIndex,
        int jointIndex,
        int axisNum)
    {
        ArticulationDrive limitDrive = axisNum == 0
            ? body.xDrive
            : body.yDrive;
        float relativeOffset =
            hasInitialJointOffsets
                ? initialJointOffsets[fingerIndex, jointIndex, axisNum]
                : 0f;
        float relativeLowerLimit = limitDrive.lowerLimit + relativeOffset;
        float relativeUpperLimit = limitDrive.upperLimit + relativeOffset;
        bool atLowerLimit = targetAngle <= limitDrive.lowerLimit;
        bool atUpperLimit = targetAngle >= limitDrive.upperLimit;

        if (!atLowerLimit)
            angleLimitWarnings[fingerIndex, jointIndex, axisNum, 0] = false;

        if (!atUpperLimit)
            angleLimitWarnings[fingerIndex, jointIndex, axisNum, 1] = false;

        string axisName = axisNum == 0 ? "X" : "Y";

        if (atLowerLimit &&
            !angleLimitWarnings[fingerIndex, jointIndex, axisNum, 0])
        {
            angleLimitWarnings[fingerIndex, jointIndex, axisNum, 0] = true;
            Debug.LogWarning(
                $"Finger {fingerIndex}, joint {jointIndex}, axis {axisName} " +
                $"target angle {relativeAngle:F2} deg reached the lower limit " +
                $"{relativeLowerLimit:F2} deg in the relative frame.",
                body);
        }

        if (atUpperLimit &&
            !angleLimitWarnings[fingerIndex, jointIndex, axisNum, 1])
        {
            angleLimitWarnings[fingerIndex, jointIndex, axisNum, 1] = true;
            Debug.LogWarning(
                $"Finger {fingerIndex}, joint {jointIndex}, axis {axisName} " +
                $"target angle {relativeAngle:F2} deg reached the upper limit " +
                $"{relativeUpperLimit:F2} deg in the relative frame.",
                body);
        }
    }

    private float GetRelativeDriveAngle(
        Transform joint,
        Vector3 incomingBone,
        Vector3 outgoingBone,
        int axisType)
    {
        if (incomingBone.sqrMagnitude <= Mathf.Epsilon ||
            outgoingBone.sqrMagnitude <= Mathf.Epsilon)
            return 0f;

        Quaternion frame = GetJointFrame(
            joint,
            incomingBone,
            Quaternion.identity);
        Vector3 localOutgoing =
            Quaternion.Inverse(frame) * outgoingBone.normalized;

        if (axisType == 0)
        {
            return Mathf.Atan2(
                -localOutgoing.y,
                -localOutgoing.z) * Mathf.Rad2Deg;
        }

        return -Mathf.Atan2(
            localOutgoing.x,
            -localOutgoing.z) * Mathf.Rad2Deg;
    }

    private Quaternion GetJointFrame(
        Transform trackedJoint,
        Vector3 incomingBone,
        Quaternion fallback)
    {
        if (incomingBone.sqrMagnitude <= Mathf.Epsilon)
            return fallback;

        Vector3 upAxis = Vector3.ProjectOnPlane(
            trackedJoint.up,
            incomingBone.normalized);
        if (upAxis.sqrMagnitude <= Mathf.Epsilon)
            return fallback;

        return Quaternion.LookRotation(
            -incomingBone.normalized,
            upAxis.normalized);
    }

    private float GetJointPosition(
        ArticulationReducedSpace position,
        int index)
    {
        return position.dofCount > index ? position[index] : 0f;
    }

    private void UpdateFingertipNode(int fingerIndex)
    {
        HapticNode node = fingertipNodes[fingerIndex];
        if (node == null)
            return;

        Vector3 physicsForce = -prevFingertipForces[fingerIndex] - fingertipInertialForces[fingerIndex];
        Vector3 fingerInertiaComponent = -(fingertipLinearMomenta[fingerIndex] - prevFingertipLinearMomenta[fingerIndex]) / Time.fixedDeltaTime;
        Vector3 rootInertiaComponent = -fingertipInertialForces[fingerIndex];
        Vector3 inertiaForce = fingerInertiaComponent + rootInertiaComponent;

        /*Debug.Log(
            $"fingerIndex: {fingerIndex}\n" +
            $"rootAccel: {rootNode.GetPhysicsAccel()}\n" +
            $"physicsForce: {physicsForce}\n" +
            $"fingerInertiaComponent: {fingerInertiaComponent}\n" +
            $"rootInertiaComponent: {rootInertiaComponent}\n" +
            $"inertiaForce: {inertiaForce}\n" +
            $"totalForce: {physicsForce - inertiaForce}\n"
        );*/

        node.UpdateState(
            physicsForce,
            inertiaForce);

        // Resets fingertip linear momenta and forces for the next frame
        prevFingertipLinearMomenta[fingerIndex] = fingertipLinearMomenta[fingerIndex];
        prevFingertipForces[fingerIndex] = fingertipForces[fingerIndex];
        prevFingertipInertialForces[fingerIndex] = fingertipInertialForces[fingerIndex];

        fingertipLinearMomenta[fingerIndex] = Vector3.zero;
        fingertipForces[fingerIndex] = Vector3.zero;
        fingertipInertialForces[fingerIndex] = Vector3.zero;
    }

    private void LogJointDriveDiagnostic(
        ArticulationBody body,
        Transform trackedJoint,
        Transform parentJoint,
        Vector3 incomingBone,
        Vector3 outgoingBone,
        Vector3 axis,
        float trackedAngle,
        float targetAngle,
        int fingerIndex,
        int jointIndex)
    {
        float actualAngle = body.jointPosition.dofCount > 0
            ? body.jointPosition[0] * Mathf.Rad2Deg
            : float.NaN;
        Transform physicsParent = body.transform.parent;
        Vector3 physicsIncoming = physicsParent == null
            ? Vector3.zero
            : body.transform.position - physicsParent.position;
        Vector3 physicsOutgoing = GetBoneVector(body.transform);
        float incomingDirectionDifference = physicsIncoming.sqrMagnitude <= Mathf.Epsilon
            ? float.NaN
            : Vector3.Angle(
                incomingBone,
                physicsIncoming);
        float outgoingDirectionDifference = physicsOutgoing.sqrMagnitude <= Mathf.Epsilon
            ? float.NaN
            : Vector3.Angle(
                outgoingBone,
                physicsOutgoing);
        float trackedAngleOnAxis = Vector3.SignedAngle(
            incomingBone,
            outgoingBone,
            axis);
        float physicsAngleOnAxis = Vector3.SignedAngle(
            physicsIncoming,
            physicsOutgoing,
            axis);
        float trackedOppositeAxisAngle = Vector3.SignedAngle(
            incomingBone,
            outgoingBone,
            -axis);
        float physicsOppositeAxisAngle = Vector3.SignedAngle(
            physicsIncoming,
            physicsOutgoing,
            -axis);
        float axisIncomingAlignment = Vector3.Dot(
            axis.normalized,
            incomingBone.normalized);
        float physicsAxisIncomingAlignment = Vector3.Dot(
            axis.normalized,
            physicsIncoming.normalized);

        Debug.Log(
            $"[HandSkeleton DRIVE DIAG] finger={fingerIndex}, joint={jointIndex}, " +
            $"physics='{body.name}', tracked='{trackedJoint.name}'\n" +
            $"target={targetAngle:F2} deg, trackedAngle={trackedAngle:F2} deg, " +
            $"actual={actualAngle:F2} deg, error={Mathf.DeltaAngle(actualAngle, targetAngle):F2} deg\n" +
            $"axis={axis}\n" +
            $"trackedIncoming={incomingBone.normalized}, " +
            $"trackedOutgoing={outgoingBone.normalized}\n" +
            $"physicsIncoming={physicsIncoming.normalized}, " +
            $"physicsOutgoing={physicsOutgoing.normalized}\n" +
            $"incomingDirectionDifference={incomingDirectionDifference:F2} deg, " +
            $"outgoingDirectionDifference={outgoingDirectionDifference:F2} deg\n" +
            $"trackedAngleOnAxis={trackedAngleOnAxis:F2} deg, " +
            $"physicsAngleOnAxis={physicsAngleOnAxis:F2} deg\n" +
            $"trackedOppositeAxis={trackedOppositeAxisAngle:F2} deg, " +
            $"physicsOppositeAxis={physicsOppositeAxisAngle:F2} deg\n" +
            $"trackedAxisIncomingDot={axisIncomingAlignment:F3}, " +
            $"physicsAxisIncomingDot={physicsAxisIncomingAlignment:F3} " +
            "(0 is perpendicular)\n" +
            $"anchorRotation={body.anchorRotation.eulerAngles}, " +
            $"limits=[{body.xDrive.lowerLimit:F2},{body.xDrive.upperLimit:F2}], " +
            $"jointPositionDof={body.jointPosition.dofCount}");
    }

    private void LogArticulationStateDiagnostics()
    {
        StringBuilder diagnostics = new StringBuilder(
            "[HandSkeleton FULL STATE DIAG]\n");
        int fingerCount = Mathf.Min(
            N_FINGERS,
            Mathf.Min(fingers.Length, trackedFingers.Length));

        for (int fingerIndex = 0; fingerIndex < fingerCount; fingerIndex++)
        {
            Transform trackedJoint = trackedFingers[fingerIndex];
            GameObject currentJoint = fingers[fingerIndex];
            diagnostics.AppendLine($"finger={fingerIndex}");

            for (int jointIndex = 0;
                 jointIndex < GetBonesPerFinger(fingerIndex) &&
                 trackedJoint != null &&
                 currentJoint != null;
                 jointIndex++)
            {
                ArticulationBody body =
                    currentJoint.GetComponent<ArticulationBody>();
                if (body == null)
                {
                    diagnostics.AppendLine(
                        $"  joint={jointIndex} physics body missing");
                    break;
                }

                Transform trackedParent = trackedJoint.parent;
                Transform physicsParent = body.transform.parent;
                Vector3 trackedIncoming = trackedParent == null
                    ? Vector3.zero
                    : trackedJoint.position - trackedParent.position;
                Vector3 physicsIncoming = physicsParent == null
                    ? Vector3.zero
                    : body.transform.position - physicsParent.position;
                Vector3 trackedOutgoing = GetBoneVector(trackedJoint);
                Vector3 physicsOutgoing = GetBoneVector(body.transform);

                diagnostics.AppendLine(
                    $"  joint={jointIndex}, body='{body.name}', " +
                    $"tracked='{trackedJoint.name}'");
                diagnostics.AppendLine(
                    $"    tracked world pos={trackedJoint.position}, " +
                    $"rot={trackedJoint.rotation.eulerAngles}");
                diagnostics.AppendLine(
                    $"    tracked local pos={trackedJoint.localPosition}, " +
                    $"rot={trackedJoint.localRotation.eulerAngles}");
                diagnostics.AppendLine(
                    $"    physics world pos={body.transform.position}, " +
                    $"rot={body.transform.rotation.eulerAngles}");
                diagnostics.AppendLine(
                    $"    physics local pos={body.transform.localPosition}, " +
                    $"rot={body.transform.localRotation.eulerAngles}");
                diagnostics.AppendLine(
                    $"    tracked incoming={trackedIncoming.normalized}, " +
                    $"outgoing={trackedOutgoing.normalized}");
                diagnostics.AppendLine(
                    $"    physics incoming={physicsIncoming.normalized}, " +
                    $"outgoing={physicsOutgoing.normalized}");
                diagnostics.AppendLine(
                    $"    direction delta incoming=" +
                    $"{GetDirectionDifference(trackedIncoming, physicsIncoming):F2} deg, " +
                    $"outgoing=" +
                    $"{GetDirectionDifference(trackedOutgoing, physicsOutgoing):F2} deg");
                diagnostics.AppendLine(
                    $"    joint position=" +
                    $"{FormatReducedSpace(body.jointPosition, true)}, " +
                    $"velocity={FormatReducedSpace(body.jointVelocity, true)} /s");
                diagnostics.AppendLine(
                    $"    xDrive target={body.xDrive.target:F2}, " +
                    $"limits=[{body.xDrive.lowerLimit:F2}," +
                    $"{body.xDrive.upperLimit:F2}], " +
                    $"yDrive target={body.yDrive.target:F2}, " +
                    $"limits=[{body.yDrive.lowerLimit:F2}," +
                    $"{body.yDrive.upperLimit:F2}]");
                diagnostics.AppendLine(
                    $"    anchor position={body.anchorPosition}, " +
                    $"rotation={body.anchorRotation.eulerAngles}");

                trackedJoint = GetBoneChild(trackedJoint);
                Transform physicsChild = GetBoneChild(body.transform);
                currentJoint = physicsChild == null
                    ? null
                    : physicsChild.gameObject;
            }
        }

        Debug.Log(diagnostics.ToString(), this);
    }

    private float GetDirectionDifference(Vector3 first, Vector3 second)
    {
        return first.sqrMagnitude <= Mathf.Epsilon ||
            second.sqrMagnitude <= Mathf.Epsilon
            ? float.NaN
            : Vector3.Angle(first, second);
    }

    private string FormatReducedSpace(
        ArticulationReducedSpace reducedSpace,
        bool convertToDegrees)
    {
        if (reducedSpace.dofCount == 0)
            return "[]";

        StringBuilder values = new StringBuilder("[");
        for (int index = 0; index < reducedSpace.dofCount; index++)
        {
            if (index > 0)
                values.Append(", ");
            float value = convertToDegrees
                ? reducedSpace[index] * Mathf.Rad2Deg
                : reducedSpace[index];
            values.Append($"{value:F3}");
        }

        values.Append(convertToDegrees ? "] deg" : "]");
        return values.ToString();
    }
}