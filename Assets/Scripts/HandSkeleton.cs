using UnityEngine;
using UnityEngine.XR.Hands;
using System.Collections.Generic;
using Unity.VisualScripting;

public class HandSkeleton : MonoBehaviour
{
    public enum Handedness
    {
        Left,
        Right
    }

    private const int N_FINGERS = 5;
    private const int N_ACTIVE_BONES = 3;
    private const string FingerColliderName = "Finger Collider";

    public Handedness handedness = Handedness.Right;
    public HapticRenderClient haptics;
    public GameObject palm;
    public Transform trackedRoot;
    public GameObject[] fingers = new GameObject[N_FINGERS];
    public Transform[] trackedFingers = new Transform[N_FINGERS];
    public float[] fingerBoneWidths = new float[3] {0.020f, 0.018f, 0.016f};
    public float[] thumbBoneWidths = new float[3] {0.025f, 0.023f, 0.021f};

    public float[] fingerJointMinimums = new float[3] {-10f, -10f, -10f};
    public float[] fingerJointMaximums = new float[3] {89f, 89f, 89f};

    public float[] thumbJointMinimums = new float[3] {-45f, -45f, -45f};
    public float[] thumbJointMaximums = new float[3] {45f, 45f, 45f};

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
    private float nextArticulationStateDiagnosticTime;

    private IntegratedHapticNode rootNode;

    private Vector3 palmColliderOffset = new Vector3(0.0047f, -0.0043f, 0.0091f);
    private Vector3 palmColliderRot = new Vector3(5.31f, 4.947f, -0.786f);
    private Vector3 palmColliderScale = new Vector3(0.07241f, 0.03259f, 0.07990f);

    private List<ArticulationBody> articulationBodies = new List<ArticulationBody>();

    // First joint has splay, second and third do not have splay
    private int[] perJointDOF = new int[3] {2, 1, 1};

    private HapticNode[] fingertipNodes = new HapticNode[N_FINGERS];
    private Vector3[] prevFingertipLinearMomenta = new Vector3[N_FINGERS];
    private Vector3[] fingertipLinearMomenta = new Vector3[N_FINGERS];
    private Vector3[] prevFingertipForces = new Vector3[N_FINGERS];
    private Vector3[] fingertipForces = new Vector3[N_FINGERS];
    private Vector3[] prevFingertipInertialForces = new Vector3[N_FINGERS];
    private Vector3[] fingertipInertialForces = new Vector3[N_FINGERS];

    // The metacarpal has no anatomical parent bone in the tracked finger
    // hierarchy. Its first observed pose is therefore the zero reference.
    private readonly float[,] metacarpalReferenceAngles = new float[N_FINGERS, 2];
    private readonly bool[,] metacarpalReferencesInitialized = new bool[N_FINGERS, 2];

    private bool prevUseGravity = false;

    void Start()
    {
        // Initialize palm articulation body and collider
        ConstructPalm();
        ConstructFingers();
    }

    void Update()
    {

    }

    void FixedUpdate()
    {
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
        for (int fingerIndex = 0; fingerIndex < Mathf.Min(N_FINGERS, fingers.Length); fingerIndex++)
        {
            GameObject currentJoint = fingers[fingerIndex];

            // Get the corresponding tracked joint transform
            Transform trackedJoint = trackedFingers[fingerIndex];

            if (trackedJoint == null || currentJoint == null)
                continue;

            for (int jointIndex = 0; jointIndex < N_ACTIVE_BONES; jointIndex++)
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
    }

    private bool IsThumb(int fingerIndex)
    {
        return fingerIndex == N_FINGERS - 1;
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

        for (int fingerIndex = 0; fingerIndex < Mathf.Min(N_FINGERS, fingers.Length); fingerIndex++)
        {
            GameObject currentJoint = fingers[fingerIndex];

            for (int jointIndex = 0; jointIndex < N_ACTIVE_BONES; jointIndex++)
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
        int dof = perJointDOF[jointIndex];

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
        body.matchAnchors = true;

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

        for (int i = 0; i < N_ACTIVE_BONES; i++)
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

    private Quaternion GetJointFrame(
        Transform trackedJoint,
        Vector3 incomingBone,
        Quaternion fallback)
    {
        if (incomingBone.sqrMagnitude <= Mathf.Epsilon)
            return fallback;

        Vector3 incomingAxis = incomingBone.normalized;
        Vector3 flexionAxis =
            Vector3.ProjectOnPlane(trackedJoint.right, incomingAxis);

        if (flexionAxis.sqrMagnitude <= Mathf.Epsilon)
            flexionAxis = Vector3.ProjectOnPlane(trackedJoint.up, incomingAxis);

        if (flexionAxis.sqrMagnitude <= Mathf.Epsilon)
            return fallback;

        flexionAxis.Normalize();
        Vector3 frameUp = Vector3.Cross(incomingAxis, flexionAxis).normalized;
        return Quaternion.LookRotation(incomingAxis, frameUp);
    }

    private void UpdateBoneCollider(int fingerIndex, int jointIndex, CapsuleCollider collider, float boneLength)
    {
        // Gets the bone width for the current joint
        float boneWidth = IsThumb(fingerIndex) ? thumbBoneWidths[jointIndex] : fingerBoneWidths[jointIndex];

        // Shortens the bone length for the last joint to be tangent to the fingertip
        if (jointIndex == N_ACTIVE_BONES - 1)
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

        // The XR hand can change both its pose and its proportions as tracking
        // confidence and the estimated skeleton change. Keep the root and bone
        // geometry aligned, while leaving joint rotation to the articulation
        // drives below.
        if (jointIndex == 0)
            AlignJointPosition(currentJoint.transform, trackedJoint);

        float boneLength = UpdateBoneLength(currentJoint.transform, trackedJoint);
        Transform colliderTransform = currentJoint.transform.Find(FingerColliderName);
        CapsuleCollider collider = colliderTransform == null
            ? null
            : colliderTransform.GetComponent<CapsuleCollider>();

        if (collider != null && boneLength > Mathf.Epsilon)
            UpdateBoneCollider(fingerIndex, jointIndex, collider, boneLength);

        float dof = perJointDOF[jointIndex];
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
                0,
                jointIndex == 0);

            if (dof > 1)
            {
                body.yDrive = UpdateAxisDrive(
                    body,
                    body.yDrive,
                    trackedJoint,
                    trackedJoint.parent,
                    fingerIndex,
                    1,
                    jointIndex == 0);

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

    private void AlignJointPosition(Transform physicsJoint, Transform trackedJoint)
    {
        if (trackedRoot == null)
            return;

        Vector3 trackedRelativePosition =
            trackedRoot.InverseTransformPoint(trackedJoint.position);
        physicsJoint.position =
            gameObject.transform.TransformPoint(trackedRelativePosition);
    }

    private float UpdateBoneLength(Transform physicsJoint, Transform trackedJoint)
    {
        Transform physicsChild = GetBoneChild(physicsJoint);
        Vector3 trackedBone = GetBoneVector(trackedJoint);
        float trackedBoneLength = trackedBone.magnitude;

        if (physicsChild == null || trackedBoneLength <= Mathf.Epsilon)
            return trackedBoneLength;

        Vector3 localDirection = physicsChild.localPosition;
        if (localDirection.sqrMagnitude <= Mathf.Epsilon)
            localDirection = Vector3.forward;
        else
            localDirection.Normalize();

        physicsChild.localPosition = localDirection * trackedBoneLength;
        return trackedBoneLength;
    }

    private ArticulationDrive UpdateAxisDrive(
        ArticulationBody body,
        ArticulationDrive drive,
        Transform trackedJoint,
        Transform parentJoint,
        int fingerIndex,
        int axisType,
        bool isMetacarpal)
    {
        if (parentJoint == null)
            return drive;

        Vector3 incomingBone = trackedJoint.position - parentJoint.position;
        Vector3 outgoingBone = GetBoneVector(trackedJoint);

        Vector3 axis = axisType == 0
            ? Utils.GetArticulationRightAxis(body)
            : Utils.GetArticulationUpAxis(body);
        float trackedAngle = GetTrackedDriveAngle(
            trackedJoint,
            incomingBone,
            outgoingBone,
            axisType);

        float targetAngle = trackedAngle;
        if (isMetacarpal)
        {
            if (!metacarpalReferencesInitialized[fingerIndex, axisType] &&
                incomingBone.sqrMagnitude > Mathf.Epsilon &&
                outgoingBone.sqrMagnitude > Mathf.Epsilon)
            {
                metacarpalReferenceAngles[fingerIndex, axisType] = trackedAngle;
                metacarpalReferencesInitialized[fingerIndex, axisType] = true;
            }

            if (metacarpalReferencesInitialized[fingerIndex, axisType])
            {
                targetAngle = Mathf.DeltaAngle(
                    metacarpalReferenceAngles[fingerIndex, axisType],
                    trackedAngle);
            }
        }
        else
        {
            Vector3 physicsOutgoing = GetBoneVector(body.transform);
            float currentAngle = body.jointPosition.dofCount > axisType
                ? body.jointPosition[axisType] * Mathf.Rad2Deg
                : 0f;

            if (physicsOutgoing.sqrMagnitude > Mathf.Epsilon &&
                outgoingBone.sqrMagnitude > Mathf.Epsilon)
            {
                // The physics bone can have a fixed local orientation offset
                // from the articulation anchor frame. Correct from the
                // measured physical bone so the target describes the actual
                // pose, not an assumed +Z rest direction.
                float geometricCorrection = Vector3.SignedAngle(
                    physicsOutgoing,
                    outgoingBone,
                    axis);
                targetAngle = currentAngle + geometricCorrection;
            }
        }

        drive.target = targetAngle;

        if (logJointDriveDiagnostics &&
            !isMetacarpal &&
            fingerIndex == diagnosticFingerIndex &&
            axisType == 0 &&
            diagnosticJointIndex >= 1 &&
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
                diagnosticJointIndex);
        }

        if (logArticulationStateDiagnostics &&
            !isMetacarpal &&
            fingerIndex == diagnosticFingerIndex &&
            axisType == 0 &&
            diagnosticJointIndex >= 1 &&
            Time.time >= nextArticulationStateDiagnosticTime)
        {
            nextArticulationStateDiagnosticTime =
                Time.time + jointDriveDiagnosticInterval;
            LogArticulationStateDiagnostic(
                body,
                trackedJoint,
                parentJoint,
                incomingBone,
                outgoingBone,
                fingerIndex,
                diagnosticJointIndex);
        }

        // Update stiffness and damping in case they have changed

        // Finds moment of inertia of current body and its children
        Utils.GetChainedMassProperties(body, out float totalMass, out Vector3 com, out Matrix4x4 totalInertia);

        // Applies coefficients to get necessary angular acceleration
        Vector3 jointPoint = body.transform.TransformPoint(body.anchorPosition);

        float axisInertia = Utils.InertiaAboutAxisAtPoint(
            totalInertia,
            totalMass,
            com,
            jointPoint,
            axis);

        drive.stiffness = fingerStiffness * axisInertia * Utils.GetMaxStiffness();
        drive.damping = fingerDamping * axisInertia * Utils.GetMaxDamping();
        drive.forceLimit = fingerMaxTorque;

        float currentPhysicsAngle = body.jointPosition[axisType] * Mathf.Rad2Deg;
        float angularError = (currentPhysicsAngle - targetAngle) * Mathf.Deg2Rad;

        float driveTorque = -(drive.stiffness * angularError + drive.damping * Vector3.Dot(body.angularVelocity, axis));

        // Clamp the drive torque to the torque limit
        driveTorque = Mathf.Clamp(driveTorque, -drive.forceLimit, drive.forceLimit);

        HapticNode fingertipNode = fingertipNodes[fingerIndex];
        if (fingertipNode == null)
            return drive;

        Vector3 jointToFingertip =
            fingertipNode.transform.position - jointPoint;

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
            Vector3.Dot(body.angularVelocity, axis);

        Vector3 linearMomentumContribution = jacobianColumn * angularMomentum;
        fingertipLinearMomenta[fingerIndex] += linearMomentumContribution;

        // Gets inertial forces at fingertips due to root node acceleration

        // Gets component of linear acceleration due to root node angular acceleration
        Vector3 rootCOMtoJointCOM = com - rootNode.GetPhysicsCOM();
        Vector3 angAccelComponent = Vector3.Cross(rootNode.GetPhysicsAngAccel(), rootCOMtoJointCOM);

        // Finds torque on the joint due to the linear acceleration and angular acceleration
        Vector3 effectiveForce = -totalMass * (rootNode.GetPhysicsAccel() + angAccelComponent);
        Vector3 jointToCOM = com - jointPoint;
        Vector3 effectiveTorque = Vector3.Cross(jointToCOM, effectiveForce);
        float effectiveJointTorque = Vector3.Dot(effectiveTorque, axis);

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

    private float GetTrackedDriveAngle(
        Transform trackedJoint,
        Vector3 incomingBone,
        Vector3 outgoingBone,
        int axisType)
    {
        if (incomingBone.sqrMagnitude <= Mathf.Epsilon ||
            outgoingBone.sqrMagnitude <= Mathf.Epsilon)
            return 0f;

        Quaternion frame = GetJointFrame(
            trackedJoint,
            incomingBone,
            Quaternion.LookRotation(incomingBone, trackedJoint.up));
        Vector3 localOutgoing =
            Quaternion.Inverse(frame) * outgoingBone.normalized;

        // The frame's +Z axis is the incoming bone. Decomposing the outgoing
        // direction in this frame gives the two articulation coordinates
        // directly, instead of measuring two coupled world-space angles.
        float denominator = Mathf.Max(Mathf.Abs(localOutgoing.z), 0.0001f);
        if (axisType == 0)
            return Mathf.Atan2(-localOutgoing.y, denominator) * Mathf.Rad2Deg;

        return Mathf.Atan2(localOutgoing.x, denominator) * Mathf.Rad2Deg;
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
        float sameAxisAngle = Vector3.SignedAngle(
            incomingBone,
            outgoingBone,
            axis);
        float oppositeAxisAngle = Vector3.SignedAngle(
            incomingBone,
            outgoingBone,
            -axis);
        float axisIncomingAlignment = Vector3.Dot(
            axis.normalized,
            incomingBone.normalized);
        Vector3 physicsIncoming = body.transform.position - parentJoint.position;
        Vector3 physicsOutgoing = GetBoneVector(body.transform);

        Debug.Log(
            $"[HandSkeleton DRIVE DIAG] finger={fingerIndex}, joint={jointIndex}, " +
            $"physics='{body.name}', tracked='{trackedJoint.name}'\n" +
            $"target={targetAngle:F2} deg, trackedAngle={trackedAngle:F2} deg, " +
            $"actual={actualAngle:F2} deg, error={Mathf.DeltaAngle(actualAngle, targetAngle):F2} deg\n" +
            $"axis={axis}, incoming={incomingBone.normalized}, outgoing={outgoingBone.normalized}\n" +
            $"sameAxis={sameAxisAngle:F2} deg, oppositeAxis={oppositeAxisAngle:F2} deg\n" +
            $"axisIncomingDot={axisIncomingAlignment:F3} " +
            $"(0 is perpendicular)\n" +
            $"physicsIncoming={physicsIncoming.normalized}, " +
            $"physicsOutgoing={physicsOutgoing.normalized}\n" +
            $"anchorRotation={body.anchorRotation.eulerAngles}, " +
            $"limits=[{body.xDrive.lowerLimit:F2},{body.xDrive.upperLimit:F2}], " +
            $"jointPositionDof={body.jointPosition.dofCount}");
    }

    private void LogArticulationStateDiagnostic(
        ArticulationBody body,
        Transform trackedJoint,
        Transform parentJoint,
        Vector3 trackedIncoming,
        Vector3 trackedOutgoing,
        int fingerIndex,
        int jointIndex)
    {
        ArticulationReducedSpace position = body.jointPosition;
        Vector3 parentAxisX = Utils.GetArticulationRightAxis(body);
        Vector3 parentAxisY = Utils.GetArticulationUpAxis(body);
        Vector3 parentAxisZ = Vector3.Cross(parentAxisX, parentAxisY).normalized;

        Vector3 parentAnchorWorld =
            body.transform.parent != null
                ? body.transform.parent.TransformPoint(body.parentAnchorPosition)
                : body.transform.TransformPoint(body.anchorPosition);
        Vector3 childAnchorWorld =
            body.transform.TransformPoint(body.anchorPosition);
        Quaternion parentFrameWorld =
            body.transform.parent != null
                ? body.transform.parent.rotation * body.parentAnchorRotation
                : body.transform.rotation * body.anchorRotation;
        Quaternion childFrameWorld =
            body.transform.rotation * body.anchorRotation;
        Quaternion relativeFrame =
            Quaternion.Inverse(parentFrameWorld) * childFrameWorld;

        Vector3 physicsIncoming =
            body.transform.position - parentJoint.position;
        Vector3 physicsOutgoing = GetBoneVector(body.transform);

        string dofs = position.dofCount == 0
            ? "none"
            : $"[{GetJointPosition(position, 0):F3}, " +
              $"{GetJointPosition(position, 1):F3}, " +
              $"{GetJointPosition(position, 2):F3}] rad";

        Debug.Log(
            $"[HandSkeleton ARTICULATION DIAG] finger={fingerIndex}, " +
            $"joint={jointIndex}, body='{body.name}', tracked='{trackedJoint.name}'\n" +
            $"trackedIncoming={trackedIncoming.normalized}, " +
            $"trackedOutgoing={trackedOutgoing.normalized}\n" +
            $"physicsIncoming={physicsIncoming.normalized}, " +
            $"physicsOutgoing={physicsOutgoing.normalized}\n" +
            $"jointPosition={dofs}, jointVelocity={body.jointVelocity}\n" +
            $"parentAnchorWorld={parentAnchorWorld}, " +
            $"childAnchorWorld={childAnchorWorld}, " +
            $"anchorDelta={(childAnchorWorld - parentAnchorWorld).magnitude:F5}\n" +
            $"parentAxisX={parentAxisX}, parentAxisY={parentAxisY}, " +
            $"parentAxisZ={parentAxisZ}\n" +
            $"parentFrame={parentFrameWorld.eulerAngles}, " +
            $"childFrame={childFrameWorld.eulerAngles}, " +
            $"relativeFrame={relativeFrame.eulerAngles}\n" +
            $"trackedJointWorld={trackedJoint.position}, " +
            $"physicsJointWorld={body.transform.position}");
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

        Vector3 physicsForce = -prevFingertipForces[fingerIndex];
        Vector3 fingerInertiaComponent = -(fingertipLinearMomenta[fingerIndex] - prevFingertipLinearMomenta[fingerIndex]) / Time.fixedDeltaTime;
        Vector3 rootInertiaComponent = fingertipInertialForces[fingerIndex];
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
}