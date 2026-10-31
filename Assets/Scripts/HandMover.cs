using UnityEngine;

public class HandMover : MonoBehaviour
{
    private Rigidbody rb;

    public float moveSpeed = 0.1f; // Speed of linear movement
    public float rotateSpeed = 100f; // Speed of rotational movement

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // Moves the hand based on keyboard input
        if (Input.GetKey(KeyCode.UpArrow))
        {
            rb.position += Vector3.up * Time.fixedDeltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            rb.position += Vector3.down * Time.fixedDeltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            rb.position += Vector3.left * Time.fixedDeltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            rb.position += Vector3.right * Time.fixedDeltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.W))
        {
            rb.position += Vector3.forward * Time.fixedDeltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.S))
        {
            rb.position += Vector3.back * Time.fixedDeltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.I))
        {
            rb.rotation *= Quaternion.AngleAxis(Time.fixedDeltaTime * rotateSpeed, Vector3.up);
        }
        if (Input.GetKey(KeyCode.O))
        {
            rb.rotation *= Quaternion.AngleAxis(Time.fixedDeltaTime * rotateSpeed, Vector3.down);
        }
        if (Input.GetKey(KeyCode.J))
        {
            rb.rotation *= Quaternion.AngleAxis(Time.fixedDeltaTime * rotateSpeed, Vector3.left);
        }
        if (Input.GetKey(KeyCode.K))
        {
            rb.rotation *= Quaternion.AngleAxis(Time.fixedDeltaTime * rotateSpeed, Vector3.right);
        }
        if (Input.GetKey(KeyCode.N))
        {
            rb.rotation *= Quaternion.AngleAxis(Time.fixedDeltaTime * rotateSpeed, Vector3.forward);
        }
        if (Input.GetKey(KeyCode.M))
        {
            rb.rotation *= Quaternion.AngleAxis(Time.fixedDeltaTime * rotateSpeed, Vector3.back);
        }
    }
}
