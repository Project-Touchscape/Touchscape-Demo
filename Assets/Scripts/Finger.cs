using UnityEngine;

public class Finger : MonoBehaviour
{
    private const int N_ACTIVE_BONES = 3;
    private GameObject[] jointObjects = new GameObject[N_ACTIVE_BONES + 1]; // +1 for the tip joint
    private HandSkeleton handSkeleton;

    // Use GameObjects so the finger walks the actual joint hierarchy instead of a Transform chain.
    public GameObject initialJoint;

    void Start()
    {
        handSkeleton = GetComponentInParent<HandSkeleton>();
        if (handSkeleton == null)
        {
            Debug.LogWarning("Finger requires a HandSkeleton in its parent hierarchy.");
            return;
        }

        if (initialJoint == null)
        {
            Debug.LogWarning("Finger initialJoint is not assigned.");
            return;
        }

        // Populate joint game objects
        jointObjects[0] = initialJoint;
        for (int i = 0; i < N_ACTIVE_BONES; i++)
        {
            jointObjects[i + 1] = GetNextJoint(jointObjects[i]);
        }

        // Construct finger joints with ArticulationBodies and CapsuleColliders
        for (int i = 0; i < N_ACTIVE_BONES; i++)
        {
            ConstructFingerJoint(jointObjects[i], jointObjects[i + 1]);
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
        if (currentJoint == null)
            return;

        // Add collider/articulation body components to joint object
        ArticulationBody body = currentJoint.GetComponent<ArticulationBody>();
        if (body == null)
        {
            body = currentJoint.AddComponent<ArticulationBody>();
        }

        CapsuleCollider capsule = currentJoint.GetComponent<CapsuleCollider>();
        if (capsule == null)
        {
            capsule = currentJoint.AddComponent<CapsuleCollider>();
        }

        // Uses distance between joint origins to determine bone length
        float boneLength = nextJoint != null
            ? Vector3.Distance(currentJoint.transform.position, nextJoint.transform.position)
            : 0.02f;

        capsule.direction = 2;
        capsule.radius = handSkeleton.boneWidth * 0.5f;
        capsule.height = boneLength + handSkeleton.boneWidth;
        capsule.center = new Vector3(0f, 0f, boneLength * 0.5f);
        capsule.material = handSkeleton.material;

        body.mass = handSkeleton.perBoneMass;
        body.anchorPosition = Vector3.zero;
        body.anchorRotation = Quaternion.identity;
        body.solverIterations = 60;
        body.solverVelocityIterations = 20;
        body.jointType = ArticulationJointType.RevoluteJoint;
        body.twistLock = ArticulationDofLock.FreeMotion;
        body.xDrive = new ArticulationDrive
        {
            stiffness = 100f * handSkeleton.strength,
            forceLimit = 1000f * handSkeleton.strength,
            damping = 3f,
            lowerLimit = -10f,
            upperLimit = 89f
        };
    }
}