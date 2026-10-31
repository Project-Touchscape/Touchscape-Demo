using UnityEngine;

public class Respawner : MonoBehaviour
{
    public float yThreshold = -1f;

    private Vector3 originalPosition;
    private Quaternion originalRotation;

    private Rigidbody rigidbody;

    void Start()
    {
        // Save the initial position and rotation.
        originalPosition = transform.position;
        originalRotation = transform.rotation;
        rigidbody = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        // Check whether the object has fallen below the threshold.
        if (transform.position.y < yThreshold)
        {
            // Restore its original position and rotation.
            transform.SetPositionAndRotation(originalPosition, originalRotation);

            // Stops any motion
            rigidbody.angularVelocity = Vector3.zero;
            rigidbody.linearVelocity = Vector3.zero;
        }
    }
}