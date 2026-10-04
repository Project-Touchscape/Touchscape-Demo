using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

public class HapticNode : MonoBehaviour
{
    //Mirror object
    public GameObject mirrorObject;

    // Game object to interact with environment
    public GameObject shadowObject;

    // Force visualization object
    public GameObject forceVisual;

    // Collision visualization object
    public GameObject collisionVisual;

    // HapticRenderClient script
    private HapticRenderClient haptics;

    //Mirror position
    private Vector3 mirrorPos = Vector3.zero;

    //Mirror orientation
    private Quaternion mirrorRot = Quaternion.identity;

    //Mirror velocity
    private Vector3 mirrorVel = Vector3.zero;

    //Mirror acceleration
    private Vector3 mirrorAccel = Vector3.zero;

    // Rigidbody of the shadow object
    private Rigidbody shadowRb;

    //Previous velocity of shadow object
    private Vector3 prevShadowVel = Vector3.zero;

    // Force to emulate with mirror object
    private Vector3 forceOnMirror = Vector3.zero;

    // Current collision candidate
    private CollisionCandidate currCandidate = new CollisionCandidate();
    private Collider shadowCollider;

    // Maximum stiffness and damping
    private float maxStiffness;
    private float maxDamping;

    // Data mutex
    private object dataLock = new object();

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // HapticNode now lives on the former shadow object. Keep the field for
        // scene compatibility, but use this GameObject when it is not assigned.
        if (shadowObject == null)
            shadowObject = gameObject;

        // Gets shadow object rigidbody
        shadowRb = GetComponent<Rigidbody>();
        shadowCollider = GetComponent<Collider>();
        // Gets maximum stiffness and damping values (would bring object to rest in one frame)
        maxStiffness = shadowRb.mass / Mathf.Pow(Time.fixedDeltaTime, 2);
        maxDamping = shadowRb.mass / Time.fixedDeltaTime;

        // Visuals start off
        forceVisual.SetActive(false);
        collisionVisual.SetActive(false);
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
        if (other.isTrigger || shadowCollider == null || shadowRb == null)
            return;

        if (CollisionCandidate.FromRaycast(shadowCollider, other, out CollisionCandidate candidate))
        {
            int result = currCandidate.CompareTo(candidate);
            if (result < 0)
                currCandidate = candidate;
            else if (result == 0)
                currCandidate.CombineWith(candidate);
        }
    }

    public void SetHaptics(HapticRenderClient haptics)
    {
        this.haptics = haptics;
    }

    public HapticRenderClient GetHaptics()
    {
        return haptics;
    }

    // Fixed update is called once per physics frame
    void FixedUpdate()
    {
        // Updates debug movement if debug mode is enabled
        UpdateDebugMovement();
        // Update kinematics of the mirror object
        UpdateKinematics();
        // Calculate the force and torque on the shadow object
        Vector3 forceOnShadow = ComputeForceOnShadow();
        Vector3 torqueOnShadow = ComputeTorqueOnShadow();
        shadowRb.AddForce(forceOnShadow);
        shadowRb.AddTorque(torqueOnShadow);

        // Calculate the force to emulate on the mirror object
        Vector3 force = -forceOnShadow;
        // Removes spring component of shadow object from the force vector
        force += GetSpringComponent();

        // If inertia is enabled, adds inertial force to the mirror
        if (haptics.inertia)
        {
            // Gets inertial force on mirror object
            Vector3 inertialForce = - shadowRb.mass * mirrorAccel / Time.fixedDeltaTime;
            // Adds inertial force to the force vector
            force += inertialForce;
        }
        
        // Clamps force to minimum
        if (force.magnitude < haptics.minForce)
        {
            force = Vector3.zero;
        }

        SetForceOnMirror(force);
        //Debug.Log($"Force on mirror: {force.magnitude}");

        // Updates force visualization
        UpdateForceVisual(force);

        // Updates collision visualization
        UpdateCollisionVisual();

        // Updates previous shadow velocity
        prevShadowVel = shadowRb.linearVelocity;

        // Publish candidates after trigger callbacks have been evaluated for this physics step.
        UpdateCollisionCandidate(currCandidate);
        currCandidate = new CollisionCandidate();
    }

    private void UpdateForceVisual(Vector3 force)
    {
        // If visualization is on and a force is present
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
        }
    }

    private void UpdateCollisionVisual()
    {
        // If visualization is on and a collision candidate is active
        if (haptics.visualization && currCandidate.IsValid())
        {
            collisionVisual.SetActive(true);
            // Positions and orients to match collision plane
            collisionVisual.transform.position = shadowObject.transform.position + currCandidate.GetCollisionPoint();
            collisionVisual.transform.rotation = Quaternion.FromToRotation(Vector3.up, currCandidate.GetCollisionNormal());
        }
        else
        {
            collisionVisual.SetActive(false);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void UpdateDebugMovement()
    {
        // Updates player movement
        if (haptics.debugMode)
        {
            // Moves mirror object in 3d with arrow keys
            if (Input.GetKey(KeyCode.UpArrow))
            {
                mirrorPos += Vector3.up * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.DownArrow))
            {
                mirrorPos += Vector3.down * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.LeftArrow))
            {
                mirrorPos += Vector3.left * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.RightArrow))
            {
                mirrorPos += Vector3.right * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.W))
            {
                mirrorPos += Vector3.forward * Time.deltaTime * haptics.debugMovementSpeed;
            }
            if (Input.GetKey(KeyCode.S))
            {
                mirrorPos += Vector3.back * Time.deltaTime * haptics.debugMovementSpeed;
            }
        }
    }

    private void SetForceOnMirror(Vector3 force)
    {
        lock (dataLock)
        {
            forceOnMirror = force;
        }
    }

    public Vector3 GetForceOnMirror()
    {
        lock (dataLock)
        {
            return forceOnMirror;
        }
    }

    public void SetMirrorPos(Vector3 pos)
    {
        lock (dataLock)
        {
            mirrorPos = pos;
        }
    }

    public Vector3 GetMirrorPos()
    {
        lock (dataLock)
        {
            return mirrorPos;
        }
    }

    public void SetMirrorRot(Quaternion rot)
    {
        lock (dataLock)
        {
            mirrorRot = rot;
        }
    }

    public Quaternion GetMirrorRot()
    {
        lock (dataLock)
        {
            return mirrorRot;
        }
    }

    private void UpdateKinematics()
    {
        // Get the mirror object's velocity and acceleration
        Vector3 mirrorPosCopy = GetMirrorPos();
        Vector3 prevMirrorPos = mirrorObject.transform.position;
        Vector3 currMirrorVel = (mirrorPosCopy - prevMirrorPos) / Time.fixedDeltaTime;
        mirrorAccel = (currMirrorVel - mirrorVel) / Time.fixedDeltaTime;
        mirrorObject.transform.position = mirrorPosCopy;
        mirrorObject.transform.rotation = GetMirrorRot();
        mirrorVel = currMirrorVel;
    }

    private Vector3 ComputeForceOnShadow()
    {
        // Gets relative position and velocities of mirror and shadow objects
        Vector3 mirrorPosCopy = mirrorObject.transform.position;
        Vector3 shadowPos = shadowObject.transform.position;

        Vector3 shadowVel = shadowRb.linearVelocity;

        Vector3 relPos = shadowPos - mirrorPosCopy;

        // Applies coefficients
        Vector3 force = -(maxStiffness * haptics.stiffness * relPos + maxDamping * haptics.damping * shadowVel);

        //Print relative position and calculated force
        //Debug.Log($"Relative position: {relPos}, force: {force}");

        return force;
    }

    private Vector3 ComputeTorqueOnShadow()
    {
        // Get orientations and angular velocities
        Quaternion mirrorRotCopy = mirrorObject.transform.rotation;
        Quaternion shadowRot = shadowObject.transform.rotation;

        Vector3 shadowAngVel = shadowRb.angularVelocity;

        // Calculate the relative rotation from mirror to shadow in global coordinates
        Vector3 relRot = AngularVelocityFromQuaternions(mirrorRotCopy, shadowRot, 1);

        // Gets maximum stiffness and damping values (would bring object to rest in one frame)
        float maxStiffness = 1 / Mathf.Pow(Time.fixedDeltaTime, 2);
        float maxDamping = 1 / Time.fixedDeltaTime;

        // Applies coefficients to get necessary angular acceleration
        Vector3 angAccel = -(maxStiffness * haptics.stiffness * relRot + maxDamping * haptics.damping * shadowAngVel);
        Vector3 axis = angAccel.normalized;

        // Finds moment of inertia
        float inertia;

        if (axis.magnitude > 0)
        {
            // Gets moment of inertia along the axis of rotation
            inertia = MomentOfInertiaAlongAxis(shadowRb, axis);
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
        //Gets previous mirror and shadow positions
        Vector3 mirrorPosCopy = mirrorObject.transform.position;
        Vector3 prevMirrorPos = mirrorPosCopy - mirrorVel * Time.fixedDeltaTime;
        Vector3 prevShadowPos = shadowObject.transform.position - shadowRb.linearVelocity * Time.fixedDeltaTime;

        //Gets predicted current shadow position and velocity
        Vector3 predShadowVel = -haptics.stiffness * (prevShadowPos - prevMirrorPos) / Time.fixedDeltaTime + prevShadowVel * (1 - haptics.damping);
        Vector3 predShadowPos = (prevMirrorPos - prevShadowPos) * haptics.stiffness + prevShadowPos + prevShadowVel * Time.fixedDeltaTime * (1 - haptics.damping);

        //Gets predicted (inertial) spring force
        Vector3 predictedForce = -(maxStiffness * haptics.stiffness * (predShadowPos - mirrorPosCopy) + maxDamping * haptics.damping * predShadowVel);

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

    private float MomentOfInertiaAlongAxis(Rigidbody rb, Vector3 axis)
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
            Vector3 selfVelocity = self.attachedRigidbody.linearVelocity;
            Vector3 otherVelocity = other.attachedRigidbody == null
                ? Vector3.zero
                : other.attachedRigidbody.linearVelocity;
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
            float selfMass = self.attachedRigidbody.mass;
            Vector3 momentumChange;

            if (other.attachedRigidbody == null)
            {
                momentumChange = 2f * selfMass *
                    Vector3.Dot(relativeVelocity, collisionNormal) * collisionNormal;
            }
            else
            {
                float otherMass = other.attachedRigidbody.mass;
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
