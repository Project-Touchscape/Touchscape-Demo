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

    /*[Header("Joint Debug")]
    public bool logSplayJoint = false;
    [Min(0)]
    public int debugFingerIndex = 0;
    [Min(0)]
    public int debugJointIndex = 0;
    [Min(0.02f)]
    public float jointLogInterval = 0.5f;
    private float nextJointLogTime;*/

    private IntegratedHapticNode palmNode;
    private HapticNode[] fingertipNodes = new HapticNode[N_FINGERS];
    private Vector3[] previousFingertipVelocities = new Vector3[N_FINGERS];
    private Vector3[] previousFingertipControllerForces = new Vector3[N_FINGERS];
    private Vector3[] fingertipControllerForces = new Vector3[N_FINGERS];
    private float[] fingertipMasses = new float[N_FINGERS];
    private Vector3[] fingertipMomenta = new Vector3[N_FINGERS];
    private bool[] initializedFingertipKinematics = new bool[N_FINGERS];

    private Vector3 palmColliderOffset = new Vector3(0.0047f, -0.0043f, 0.0091f);
    private Vector3 palmColliderRot = new Vector3(5.31f, 4.947f, -0.786f);
    private Vector3 palmColliderScale = new Vector3(0.07241f, 0.03259f, 0.07990f);

    // Cache previous gravity state to detect changes
    private bool prevGravity = false;
    
    private float maxStiffness;
    private float maxDamping;

    private XRHand hand;
    private List<ArticulationBody> articulationBodies = new List<ArticulationBody>();
    private List<float> initialJointAngles = new List<float>();
    private float[] initialSplayAngles = new float[N_FINGERS];

    // First joint has splay, second and third do not have splay
    private int[] perJointDOF = new int[3] {2, 1, 1};

    private float totalMass;

    void Start()
    {
        // Set initial gravity state
        prevGravity = useGravity;

        // Gets maximum stiffness and damping coefficients (would bring object to rest in one frame)
        maxStiffness = 1 / Mathf.Pow(Time.fixedDeltaTime, 2);
        maxDamping = 1 / Time.fixedDeltaTime;

        // Calculate total mass
        totalMass = palmMass + (perBoneMass * N_FINGERS * N_ACTIVE_BONES);
        
        // Initialize palm articulation body and collider
        ConstructPalm();
        ConstructFingers();
    }

    void Update()
    {
        
    }

    void FixedUpdate()
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

        int jointID = 0;
        System.Array.Clear(fingertipMasses, 0, fingertipMasses.Length);
        System.Array.Clear(fingertipMomenta, 0, fingertipMomenta.Length);
        System.Array.Clear(fingertipControllerForces, 0, fingertipControllerForces.Length);
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
                // Update the articulation body's target angle based on the tracked joint's rotation
                UpdateJointParams(
                    fingerIndex,
                    jointID,
                    jointIndex,
                    currentJoint,
                    trackedJoint);

                // Move to the next joint in the finger hierarchy
                currentJoint = currentJoint.transform.GetChild(0).gameObject;
                trackedJoint = trackedJoint.GetChild(0);
                jointID++;
            }
        }

        // Update palm haptic node parameters
        if (palmNode != null)
        {
            palmNode.posStiffness = palmPosStiffness;
            palmNode.posDamping = palmPosDamping;
            palmNode.rotStiffness = palmRotStiffness;
            palmNode.rotDamping = palmRotDamping;
            palmNode.maxForce = palmMaxForce;
            palmNode.maxTorque = palmMaxTorque;
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
        if (!gameObject.TryGetComponent<IntegratedHapticNode>(out palmNode))
            palmNode = gameObject.AddComponent<IntegratedHapticNode>();
        palmNode.haptics = haptics;
        palmNode.trackedTransform = trackedRoot;
        palmNode.useChainedMass = true;
        palmNode.posStiffness = palmPosStiffness;
        palmNode.posDamping = palmPosDamping;
        palmNode.rotStiffness = palmRotStiffness;
        palmNode.rotDamping = palmRotDamping;
        // Normalizes max force by total mass
        palmNode.maxForce = palmMaxForce;
        palmNode.maxTorque = palmMaxTorque;

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
        palmBody.collisionDetectionMode = CollisionDetectionMode.Continuous;
        articulationBodies.Add(palmBody);

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

    private bool IsThumb(int fingerIndex)
    {
        return fingerIndex == N_FINGERS - 1;
    }

    private void ConstructFingerJoint(int fingerIndex, int jointIndex, GameObject currentJoint)
    {
        // Fetch per joint properties
        bool isThumb = IsThumb(fingerIndex);
        // Thumb has one less joint so skip first one
        int dof = perJointDOF[jointIndex];

        Vector3 boneDirection = GetBoneVector(currentJoint.transform);

        // Put the collider on a child so its rotation can be set independently of the
        // tracked joint's parent orientation.
        Transform colliderTransform = currentJoint.transform.Find(FingerColliderName);
        if (colliderTransform == null)
        {
            GameObject colliderObject = new GameObject(FingerColliderName);
            colliderTransform = colliderObject.transform;
            colliderTransform.SetParent(currentJoint.transform, false);
        }
        colliderTransform.gameObject.layer = currentJoint.layer;

        // Points child object z-axis in the direction of the next joint, or forward if there is no next joint.
        colliderTransform.localPosition = Vector3.zero;
        if (boneDirection.sqrMagnitude > Mathf.Epsilon)
            colliderTransform.rotation = Quaternion.LookRotation(boneDirection.normalized, currentJoint.transform.up);

        // Adds capsule collider to the child object
        CapsuleCollider capsule = colliderTransform.GetComponent<CapsuleCollider>();
        if (capsule == null)
            capsule = colliderTransform.gameObject.AddComponent<CapsuleCollider>();

        UpdateBoneCollider(isThumb, jointIndex, capsule, boneDirection.magnitude);

        // Add the articulation body to the current joint object
        if (!currentJoint.TryGetComponent<ArticulationBody>(out var body))
            body = currentJoint.AddComponent<ArticulationBody>();

        body.mass = perBoneMass;
        body.useGravity = useGravity;
        body.anchorPosition = Vector3.zero;
        body.collisionDetectionMode = CollisionDetectionMode.Continuous;
        // Anchor rotations are expressed in the articulation body's local frame.
        // Convert the bone direction to that frame before aligning the anchor's
        // local forward axis with the bone.
        Vector3 localBoneDirection = currentJoint.transform.InverseTransformDirection(boneDirection.normalized);
        body.anchorRotation = Quaternion.FromToRotation(Vector3.forward, localBoneDirection);
        Vector3 parentBoneDirection = GetBoneVector(currentJoint.transform.parent);
            

        body.solverIterations = 60;
        body.solverVelocityIterations = 20;
        if (dof > 0)
        {

            float initialAngle = Vector3.SignedAngle(parentBoneDirection, boneDirection, Utils.GetArticulationRightAxis(body));
            initialJointAngles.Add(initialAngle);

            body.jointType = dof == 1 ? ArticulationJointType.RevoluteJoint : ArticulationJointType.SphericalJoint;
            body.twistLock = ArticulationDofLock.LimitedMotion;
            float lowerLimit = isThumb ? thumbJointMinimums[jointIndex] : fingerJointMinimums[jointIndex];
            float upperLimit = isThumb ? thumbJointMaximums[jointIndex] : fingerJointMaximums[jointIndex];
            // Note: limits are referenced to initial angle so that parallel with parent bone is zero
            lowerLimit -= initialAngle;
            upperLimit -= initialAngle;
            
            body.xDrive = new ArticulationDrive
            {
                // Note: positive angles are flexion, negative angles are extension. The initial angle is subtracted from the target to make the joint's current position the zero point for the drive.
                driveType = ArticulationDriveType.Force,
                target = 0f,
                stiffness = 0f,
                forceLimit = 0f,
                damping = 0f,
                lowerLimit = lowerLimit,
                upperLimit = upperLimit,
            };

            if (dof > 1)
            {
                // Creates splay drive
                float initialSplayAngle = Vector3.SignedAngle(parentBoneDirection, boneDirection, Utils.GetArticulationUpAxis(body));
                initialSplayAngles[fingerIndex] = initialSplayAngle;

                body.swingYLock = ArticulationDofLock.LimitedMotion;
                body.swingZLock = ArticulationDofLock.LockedMotion;
                float splayLowerLimit = isThumb ? thumbSplayMinimum : fingerSplayMinimum;
                float splayUpperLimit = isThumb ? thumbSplayMaximum : fingerSplayMaximum;

                body.yDrive = new ArticulationDrive
                {
                    driveType = ArticulationDriveType.Force,
                    target = 0f,
                    stiffness = 0f,
                    forceLimit = 0f,
                    damping = 0f,
                    lowerLimit = splayLowerLimit,
                    upperLimit = splayUpperLimit,
                };
            }
        }
        else
        {
            body.jointType = ArticulationJointType.FixedJoint;
        }
        articulationBodies.Add(body);

        // Adds haptic node if it is the last joint
        if (jointIndex == N_ACTIVE_BONES - 1)
        {
            if (!currentJoint.TryGetComponent<HapticNode>(out HapticNode node))
                node = currentJoint.AddComponent<HapticNode>();

            // Gets tracked joint
            Transform trackedJoint = trackedFingers[fingerIndex];
            for (int i = 0; i < N_ACTIVE_BONES; i++)
            {
                trackedJoint = trackedJoint.childCount > 0 ? trackedJoint.GetChild(0) : null;
            }

            node.haptics = haptics;
            node.trackedTransform = trackedJoint;
            node.useInertia = false;
            fingertipNodes[fingerIndex] = node;
        }
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

    private void UpdateBoneCollider(bool isThumb, int jointIndex, CapsuleCollider collider, float boneLength)
    {
        // Gets the bone width for the current joint
        float boneWidth = isThumb ? thumbBoneWidths[jointIndex] : fingerBoneWidths[jointIndex];

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
        int jointID,
        int jointIndex,
        GameObject currentJoint,
        Transform trackedJoint)
    {
        if (currentJoint == null || trackedJoint == null)
            return;

        // Ensures metacarpal joint position is aligned with the tracked hand
        if (jointIndex == 0)
        {
            alignJointPosition(currentJoint.transform, trackedJoint);
        }

        // Updates other joint positions and collider to account for potential changes in bone length
        float boneLength = updateBoneLength(currentJoint.transform, trackedJoint);
        Transform colliderTransform = currentJoint.transform.Find(FingerColliderName);
        CapsuleCollider collider = colliderTransform == null
            ? null
            : colliderTransform.GetComponent<CapsuleCollider>();
        if (collider != null)
        {
            UpdateBoneCollider(IsThumb(fingerIndex), jointIndex, collider, boneLength);
        }

        float dof = perJointDOF[jointIndex];
        ArticulationBody body = currentJoint.GetComponent<ArticulationBody>();
        if (body == null)
        {
            return;
        }

        fingertipMasses[fingerIndex] += body.mass;
        fingertipMomenta[fingerIndex] += body.mass * body.linearVelocity;

        if (dof > 0)
        {
            // Measure the tracked bones around the physics joint's local X axis. The
            // articulation drive rotates around this axis, and using trackedJoint.right
            // here can introduce a thumb flexion offset when the two rigs use different
            // local axes.
            float initialAngle = initialJointAngles[jointID];
            Vector3 trackedBoneDirection = GetBoneVector(trackedJoint);
            Vector3 parentBoneDirection = GetBoneVector(trackedJoint.parent);
            Vector3 rightAxis = Utils.GetArticulationRightAxis(body);
            
            body.xDrive = UpdateAxisDrive(
                body,
                body.xDrive,
                rightAxis,
                initialAngle,
                parentBoneDirection,
                trackedBoneDirection,
                fingerIndex,
                0);
            
            if (dof > 1)
            {
                float initialSplayAngle = initialSplayAngles[fingerIndex];
                Vector3 upAxis = Utils.GetArticulationUpAxis(body);
                body.yDrive = UpdateAxisDrive(
                    body,
                    body.yDrive,
                    upAxis,
                    initialSplayAngle,
                    parentBoneDirection,
                    trackedBoneDirection,
                    fingerIndex,
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
                        upAxis,
                        initialAngle,
                        initialSplayAngle);
                }*/
            }
        }

        if (jointIndex == N_ACTIVE_BONES - 1 &&
            currentJoint.transform.childCount > 0 &&
            trackedJoint.childCount > 0)
        {
            UpdateFingertipWrench(
                fingerIndex,
                currentJoint.transform.GetChild(0));
        }
    }

    private void alignJointPosition(Transform physicsJoint, Transform trackedJoint)
    {
        Vector3 trackedRelPos = trackedRoot.InverseTransformPoint(trackedJoint.position);
        physicsJoint.position = gameObject.transform.TransformPoint(trackedRelPos);
    }

    private float updateBoneLength(Transform physicsJoint, Transform trackedJoint)
    {
        float trackedBoneLength = GetBoneVector(trackedJoint).magnitude;
        float physicsBoneLength = GetBoneVector(physicsJoint).magnitude;
        Transform child = physicsJoint.GetChild(0);
        child.localPosition *= trackedBoneLength / physicsBoneLength;
        return trackedBoneLength;
    }

    private void LogSplayJoint(
        int fingerIndex,
        int jointIndex,
        GameObject currentJoint,
        Transform trackedJoint,
        ArticulationBody body,
        Vector3 trackedParentBoneDirection,
        Vector3 trackedBoneDirection,
        Vector3 flexionAxis,
        Vector3 splayAxis,
        float initialFlexionAngle,
        float initialSplayAngle)
    {
        Vector3 physicsBoneDirection = GetBoneVector(currentJoint.transform);
        Vector3 physicsParentBoneDirection = GetBoneVector(
            currentJoint.transform.parent);
        float currentFlexionAngle = Vector3.SignedAngle(
            physicsParentBoneDirection,
            physicsBoneDirection,
            flexionAxis) - initialFlexionAngle;
        float currentSplayAngle = Vector3.SignedAngle(
            physicsParentBoneDirection,
            physicsBoneDirection,
            splayAxis) - initialSplayAngle;
        float targetFlexionAngle = Vector3.SignedAngle(
            trackedParentBoneDirection,
            trackedBoneDirection,
            flexionAxis) - initialFlexionAngle;
        float targetSplayAngle = Vector3.SignedAngle(
            trackedParentBoneDirection,
            trackedBoneDirection,
            splayAxis) - initialSplayAngle;
        ArticulationDrive flexionDrive = body.xDrive;
        ArticulationDrive splayDrive = body.yDrive;

        Debug.Log(
            $"[HandSkeleton] Splay joint debug\n" +
            $"Finger index: {fingerIndex}, joint index: {jointIndex}\n" +
            $"Physics joint: {currentJoint.name}, tracked joint: {trackedJoint.name}\n" +
            $"Physics position: {currentJoint.transform.position}, " +
            $"tracked position: {trackedJoint.position}\n" +
            $"Physics bone: {physicsBoneDirection}, " +
            $"tracked bone: {trackedBoneDirection}\n" +
            $"Physics parent bone: {physicsParentBoneDirection}, " +
            $"tracked parent bone: {trackedParentBoneDirection}\n" +
            $"Flexion axis: {flexionAxis}, splay axis: {splayAxis}\n" +
            $"Flexion current: {currentFlexionAngle:F3}, " +
            $"flexion target: {targetFlexionAngle:F3}, " +
            $"flexion drive target: {flexionDrive.target:F3}\n" +
            $"Splay current: {currentSplayAngle:F3}, " +
            $"splay target: {targetSplayAngle:F3}, " +
            $"splay drive target: {splayDrive.target:F3}\n" +
            $"Initial flexion: {initialFlexionAngle:F3}, " +
            $"initial splay: {initialSplayAngle:F3}\n"
            );
    }

    private ArticulationDrive UpdateAxisDrive(
        ArticulationBody body,
        ArticulationDrive drive,
        Vector3 axis,
        float initialAngle,
        Vector3 parentBoneDirection,
        Vector3 trackedBoneDirection,
        int fingerIndex,
        int jointPositionIndex)
    {
        // Note: positive angles are flexion, negative angles are extension. The initial angle is subtracted from the target to make the joint's current position the zero point for the drive.
        float currentAngle = Vector3.SignedAngle(
            parentBoneDirection,
            trackedBoneDirection,
            axis);
        float targetAngle = currentAngle - initialAngle;
        drive.target = targetAngle;
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
        drive.stiffness = fingerStiffness * axisInertia * maxStiffness;
        drive.damping = fingerDamping * axisInertia * maxDamping;


        ArticulationReducedSpace jointPosition = body.jointPosition;
        float currentPhysicsAngle =
            jointPosition[jointPositionIndex] * Mathf.Rad2Deg;
        float angularError = targetAngle - currentPhysicsAngle;
        float driveTorque =
            drive.stiffness * angularError
            - drive.damping * Vector3.Dot(body.angularVelocity, axis);

        HapticNode fingertipNode = fingertipNodes[fingerIndex];
        Vector3 jointToFingertip =
            fingertipNode.transform.position - jointPoint;
        if (jointToFingertip.sqrMagnitude > Mathf.Epsilon)
        {
            Vector3 torque = axis * driveTorque;
            Vector3 fingertipForce =
                Vector3.Cross(torque, jointToFingertip)
                / jointToFingertip.sqrMagnitude;
            fingertipControllerForces[fingerIndex] +=
                fingertipForce;
        }

        // Updates force limit based on effective radius
        drive.forceLimit = Mathf.Sqrt(axisInertia / totalMass) * palmMaxForce;
        //drive.forceLimit = jointToFingertip.magnitude * palmMaxForce;

        Debug.Log(
            $"fingerIndex: {fingerIndex}, jointPositionIndex: {jointPositionIndex}, " +
            $"axisInertia: {axisInertia:F3}, totalMass: {totalMass:F3}, com: {com}, jointPoint: {jointPoint}, " +
            $"targetAngle: {targetAngle:F3}, currentPhysicsAngle: {currentPhysicsAngle:F3}, angularError: {angularError:F3}, driveTorque: {driveTorque:F3}");
        
        return drive;
    }

    private void UpdateFingertipWrench(
        int fingerIndex,
        Transform fingertip)
    {
        HapticNode node = fingertipNodes[fingerIndex];
        if (node == null)
            return;

        float timeStep = Time.fixedDeltaTime;
        float fingertipMass = fingertipMasses[fingerIndex];
        if (fingertipMass <= Mathf.Epsilon)
            return;

        Vector3 fingertipVelocity =
            fingertipMomenta[fingerIndex] / fingertipMass;
        Vector3 fingertipAcceleration = initializedFingertipKinematics[fingerIndex]
            ? (fingertipVelocity - previousFingertipVelocities[fingerIndex]) / timeStep
            : Vector3.zero;

        Vector3 externalForce =
            fingertipMass * fingertipAcceleration
            - previousFingertipControllerForces[fingerIndex];

        node.UpdateWrench(
            -externalForce,
            Vector3.zero,
            Vector3.zero);

        previousFingertipVelocities[fingerIndex] = fingertipVelocity;
        previousFingertipControllerForces[fingerIndex] =
            fingertipControllerForces[fingerIndex];
        fingertipControllerForces[fingerIndex] = Vector3.zero;
        initializedFingertipKinematics[fingerIndex] = true;
    }
}
