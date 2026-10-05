using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

public class HapticNode : MonoBehaviour
{
    // HapticRenderClient script
    public HapticRenderClient haptics;

    // Tracking object
    public Transform trackedTransform;

    // Force visualization object
    public GameObject forceVisual;

    // Collision visualization object
    public GameObject collisionVisual;

    //Tracked position
    private Vector3 trackedPos = Vector3.zero;

    //Tracked orientation
    private Quaternion trackedRot = Quaternion.identity;

    //Tracked velocity
    private Vector3 trackedVel = Vector3.zero;

    //Tracked acceleration
    private Vector3 trackedAccel = Vector3.zero;

    // ArticulationBody of the physics object
    private ArticulationBody physicsAb;

    //Previous velocity of physics object
    private Vector3 prevPhysicsVel = Vector3.zero;

    // Force to emulate with tracked object
    private Vector3 forceOnTracked = Vector3.zero;

    // Current collision candidate
    private CollisionCandidate currCandidate = new CollisionCandidate();
    private Collider physicsCollider;

    // Maximum stiffness and damping
    private float maxStiffness;
    private float maxDamping;

    // Data mutex
    private object dataLock = new object();

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // Gets physics object rigidbody
        physicsAb = gameObject.GetComponent<ArticulationBody>();
        physicsCollider = gameObject.GetComponent<Collider>();
        // Gets maximum stiffness and damping values (would bring object to rest in one frame)
        maxStiffness = physicsAb.mass / Mathf.Pow(Time.fixedDeltaTime, 2);
        maxDamping = physicsAb.mass / Time.fixedDeltaTime;

        // Visuals start off
        //forceVisual.SetActive(false);
        //collisionVisual.SetActive(false);
    }

    void OnTriggerEnter(Collider other)
    {
        EvaluateTrigger(other);
    }

    void OnTriggerStay(Collider other)
    {
        EvaluateTrigger(other);
    }

    private void EvaluateTrigger(Collider other)
    {
        if (other.isTrigger || physicsCollider == null || physicsAb == null)
            return;

        if (CollisionCandidate.FromRaycast(physicsCollider, other, out CollisionCandidate candidate))
        {
            int result = currCandidate.CompareTo(candidate);
            if (result < 0)
                currCandidate = candidate;
            else if (result == 0)
                currCandidate.CombineWith(candidate);
        }
    }

    // Fixed update is called once per physics frame
    void FixedUpdate()
    {
        // Updates debug movement if debug mode is enabled
        UpdateDebugMovement();
        // Update kinematics of the tracked object
        UpdateKinematics();
        // Calculate the force and torque on the physics object
        Vector3 forceOnPhysics = ComputeForceOnPhysics();
        Vector3 torqueOnPhysics = ComputeTorqueOnPhysics();
        physicsAb.AddForce(forceOnPhysics);
        physicsAb.AddTorque(torqueOnPhysics);

        // Calculate the force to emulate on the tracked object
        Vector3 force = -forceOnPhysics;
        // Removes spring component of physics object from the force vector
        force += GetSpringComponent();

        // If inertia is enabled, adds inertial force to the tracked
        if (haptics.inertia)
        {
            // Gets inertial force on tracked object
            Vector3 inertialForce = - physicsAb.mass * trackedAccel / Time.fixedDeltaTime;
            // Adds inertial force to the force vector
            force += inertialForce;
        }
        
        // Clamps force to minimum
        if (force.magnitude < haptics.minForce)
        {
            force = Vector3.zero;
        }

        SetForceOnTracked(force);
        //Debug.Log($"Force on tracked: {force.magnitude}");

        // Updates force visualization
        UpdateForceVisual(force);

        // Updates collision visualization
        UpdateCollisionVisual();

        // Updates previous physics velocity
        prevPhysicsVel = physicsAb.linearVelocity;

        // Publish candidates after trigger callbacks have been evaluated for this physics step.
        UpdateCollisionCandidate(currCandidate);
        currCandidate = new CollisionCandidate();
    }

    private void UpdateForceVisual(Vector3 force)
    {
        /*// If visualization is on and a force is present
        if (haptics.visualization && force.magnitude > 0)
        {
            forceVisual.SetActive(true);
            //Scales and orients force visual by force
            forceVisual.transform.rotation = Quaternion.FromToRotation(Vector3.up, force.normalized);
            forceVisual.transform.localScale = new Vector3(1f, force.magnitude * haptics.forceVisualScale, 1f);
        }
        else
        {
            forceVisual.SetActive(false);
        }*/
    }

    private void UpdateCollisionVisual()
    {
        /*// If visualization is on and a collision candidate is active
        if (haptics.visualization && currCandidate.IsValid())
        {
            collisionVisual.SetActive(true);
            // Positions and orients to match collision plane
            collisionVisual.transform.position = gameObject.transform.position + currCandidate.GetCollisionPoint();
            collisionVisual.transform.rotation = Quaternion.FromToRotation(Vector3.up, currCandidate.GetCollisionNormal());
        }
        else
        {
            collisionVisual.SetActive(false);
        }*/
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void UpdateDebugMovement()
    {
        // Updates player movement
        /*if (haptics.debugMode)
        {
            // Moves tracked object in 3d with arrow keys
            if (Input.GetKey(KeyCode.UpArrow))
            {
                trackedPos += Vector3.up * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.DownArrow))
            {
                trackedPos += Vector3.down * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.LeftArrow))
            {
                trackedPos += Vector3.left * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.RightArrow))
            {
                trackedPos += Vector3.right * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.W))
            {
                trackedPos += Vector3.forward * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.S))
            {
                trackedPos += Vector3.back * Time.deltaTime * haptics.debugMovementSpeed;
            }
        }*/
    }

    private void SetForceOnTracked(Vector3 force)
    {
        lock (dataLock)
        {
            forceOnTracked = force;
        }
    }

    public Vector3 GetForceOnTracked()
    {
        lock (dataLock)
        {
            return forceOnTracked;
        }
    }

    public void SetTrackedPos(Vector3 pos)
    {
        lock (dataLock)
        {
            trackedPos = pos;
        }
    }

    public Vector3 GetTrackedPos()
    {
        lock (dataLock)
        {
            return trackedPos;
        }
    }

    public void SetTrackedRot(Quaternion rot)
    {
        lock (dataLock)
        {
            trackedRot = rot;
        }
    }

    public Quaternion GetTrackedRot()
    {
        lock (dataLock)
        {
            return trackedRot;
        }
    }

    private void UpdateKinematics()
    {
        // Get the tracked object's velocity and acceleration
        Vector3 trackedPosCopy = trackedTransform.position;
        Vector3 prevTrackedPos = GetTrackedPos();
        Vector3 currTrackedVel = (trackedPosCopy - prevTrackedPos) / Time.fixedDeltaTime;
        trackedAccel = (currTrackedVel - trackedVel) / Time.fixedDeltaTime;
        SetTrackedPos(trackedPosCopy);
        SetTrackedRot(trackedTransform.rotation);
        trackedVel = currTrackedVel;
    }

    private Vector3 ComputeForceOnPhysics()
    {
        // Gets relative position and velocities of tracked and physics objects
        Vector3 trackedPosCopy = GetTrackedPos();
        Vector3 physicsPos = gameObject.transform.position;

        Vector3 physicsVel = physicsAb.linearVelocity;

        Vector3 relPos = physicsPos - trackedPosCopy;

        // Applies coefficients
        Vector3 force = -(maxStiffness * haptics.stiffness * relPos + maxDamping * haptics.damping * physicsVel);

        //Print relative position and calculated force
        //Debug.Log($"Relative position: {relPos}, force: {force}");

        return force;
    }

    private Vector3 ComputeTorqueOnPhysics()
    {
        // Get orientations and angular velocities
        Quaternion trackedRotCopy = GetTrackedRot();
        Quaternion physicsRot = gameObject.transform.rotation;

        Vector3 physicsAngVel = physicsAb.angularVelocity;

        // Calculate the relative rotation from tracked to physics in global coordinates
        Vector3 relRot = AngularVelocityFromQuaternions(trackedRotCopy, physicsRot, 1);

        // Gets maximum stiffness and damping values (would bring object to rest in one frame)
        float maxStiffness = 1 / Mathf.Pow(Time.fixedDeltaTime, 2);
        float maxDamping = 1 / Time.fixedDeltaTime;

        // Applies coefficients to get necessary angular acceleration
        Vector3 angAccel = -(maxStiffness * haptics.stiffness * relRot + maxDamping * haptics.damping * physicsAngVel);
        Vector3 axis = angAccel.normalized;

        // Finds moment of inertia
        float inertia;

        if (axis.magnitude > 0)
        {
            // Gets moment of inertia along the axis of rotation
            inertia = MomentOfInertiaAlongAxis(physicsAb, axis);
        }
        else
        {
            inertia = 0;
        }

        Vector3 torque = angAccel * inertia;

        // Print relative rotation and calculated torque
        //Debug.Log($"Relative rotation axis: {relRot.normalized}, angle: {relRot.magnitude} rad, torque: {torque}");

        return torque;
    }

    private Vector3 GetSpringComponent()
    {
        //Gets previous tracked and physics positions
        Vector3 trackedPosCopy = GetTrackedPos();
        Vector3 prevTrackedPos = trackedPosCopy - trackedVel * Time.fixedDeltaTime;
        Vector3 prevPhysicsPos = gameObject.transform.position - physicsAb.linearVelocity * Time.fixedDeltaTime;

        //Gets predicted current physics position and velocity
        Vector3 predPhysicsVel = -haptics.stiffness * (prevPhysicsPos - prevTrackedPos) / Time.fixedDeltaTime + prevPhysicsVel * (1 - haptics.damping);
        Vector3 predPhysicsPos = (prevTrackedPos - prevPhysicsPos) * haptics.stiffness + prevPhysicsPos + prevPhysicsVel * Time.fixedDeltaTime * (1 - haptics.damping);

        //Gets predicted (inertial) spring force
        Vector3 predictedForce = -(maxStiffness * haptics.stiffness * (predPhysicsPos - trackedPosCopy) + maxDamping * haptics.damping * predPhysicsVel);

        return predictedForce;
    }

    public void UpdateCollisionCandidate(CollisionCandidate candidate)
    {
        // Sends collision candidate to haptic client if it is valid
        if (candidate.IsValid())
        {
            haptics.SendCollisionCandidate(candidate);
        }

        // Updates current collision candidate using copy constructor
        currCandidate = new CollisionCandidate(candidate);
    }

    private float MomentOfInertiaAlongAxis(ArticulationBody rb, Vector3 axis)
    {
        axis = Quaternion.Inverse(rb.inertiaTensorRotation) * axis.normalized; //rotating the torque because it’s equivalent and more efficient
        Vector3 angularAcceleration = new Vector3(Vector3.Dot(Vector3.right, axis) / rb.inertiaTensor.x, Vector3.Dot(Vector3.up, axis) / rb.inertiaTensor.y, Vector3.Dot(Vector3.forward, axis) / rb.inertiaTensor.z); //calculating the angular acceleration that would result from a torque of 1 Nm (the same way that unity does it)
        return 1 / angularAcceleration.magnitude; //moment of inertia = Torque / angular acceleration
    }

    private Vector3 AngularVelocityFromQuaternions(Quaternion q1, Quaternion q2, float timeStep)
    {
        // Calculate the angular velocity from two quaternions
        Quaternion delta = Quaternion.Inverse(q1) * q2;
        delta.ToAngleAxis(out float angle, out Vector3 axis);
        if (angle > 180f) angle -= 360f; // Ensure shortest path
        return axis * (angle * Mathf.Deg2Rad / timeStep);
    }

    private void OnApplicationQuit()
    {
        
    }

    public class CollisionCandidate
    {
        private Vector3 collisionPoint = Vector3.zero;
        private Vector3 collisionNormal = Vector3.zero;
        private Vector3 momentumChange = Vector3.zero;
        private float timeUntilCollision;

        public CollisionCandidate()
        {
        }

        public CollisionCandidate(CollisionCandidate other)
            : this(other.GetCollisionPoint(), other.GetCollisionNormal(),
                other.GetMomentumChange(), other.GetTimeUntilCollision())
        {
        }

        private CollisionCandidate(
            Vector3 collisionPoint,
            Vector3 collisionNormal,
            Vector3 momentumChange,
            float timeUntilCollision)
        {
            this.collisionPoint = collisionPoint;
            this.collisionNormal = collisionNormal;
            this.momentumChange = momentumChange;
            this.timeUntilCollision = timeUntilCollision;
        }

        public static bool FromRaycast(Collider self, Collider other, out CollisionCandidate result)
        {
            Vector3 selfVelocity = self.attachedArticulationBody.linearVelocity;
            Vector3 otherVelocity = other.attachedArticulationBody == null
                ? Vector3.zero
                : other.attachedArticulationBody.linearVelocity;
            Vector3 relativeVelocity = selfVelocity - otherVelocity;

            if (relativeVelocity.sqrMagnitude <= Mathf.Epsilon)
            {
                result = null;
                return false;
            }

            Vector3 direction = relativeVelocity.normalized;
            if (!other.Raycast(new Ray(self.bounds.center, direction), out RaycastHit selfToOther, Mathf.Infinity) ||
                !self.Raycast(new Ray(selfToOther.point, -direction), out RaycastHit contactToSelf, Mathf.Infinity))
            {
                result = null;
                return false;
            }

            float timeUntilCollision = contactToSelf.distance / relativeVelocity.magnitude;
            Vector3 collisionPoint = selfVelocity * timeUntilCollision;
            Vector3 collisionNormal = selfToOther.normal;
            float selfMass = self.attachedArticulationBody.mass;
            Vector3 momentumChange;

            if (other.attachedArticulationBody == null)
            {
                momentumChange = 2f * selfMass *
                    Vector3.Dot(relativeVelocity, collisionNormal) * collisionNormal;
            }
            else
            {
                float otherMass = other.attachedArticulationBody.mass;
                float initialSelfVelocity = Vector3.Dot(selfVelocity, collisionNormal);
                float initialOtherVelocity = Vector3.Dot(otherVelocity, collisionNormal);
                float finalSelfVelocity =
                    (2f * otherMass * initialOtherVelocity +
                        (selfMass - otherMass) * initialSelfVelocity) /
                    (selfMass + otherMass);
                momentumChange = selfMass *
                    (finalSelfVelocity - initialSelfVelocity) * collisionNormal;
            }

            result = new CollisionCandidate(
                collisionPoint, collisionNormal, momentumChange, timeUntilCollision);
            return true;
        }

        public float GetTimeUntilCollision()
        {
            return timeUntilCollision;
        }

        public Vector3 GetCollisionPoint()
        {
            return collisionPoint;
        }

        public Vector3 GetCollisionNormal()
        {
            return collisionNormal;
        }

        public Vector3 GetMomentumChange()
        {
            return momentumChange;
        }

        public bool IsValid()
        {
            return momentumChange.sqrMagnitude > 0f;
        }

        public int CompareTo(CollisionCandidate other)
        {
            if (!IsValid())
                return -1;
            if (!other.IsValid())
                return 1;

            float time = GetTimeUntilCollision();
            float otherTime = other.GetTimeUntilCollision();
            if (otherTime >= 2f * time)
                return 1;
            if (time >= 2f * otherTime)
                return -1;
            return 0;
        }

        public void CombineWith(CollisionCandidate other)
        {
            momentumChange += other.GetMomentumChange();
            if (other.GetTimeUntilCollision() < GetTimeUntilCollision())
            {
                timeUntilCollision = other.GetTimeUntilCollision();
                collisionPoint = other.GetCollisionPoint();
            }
        }
    }
}
