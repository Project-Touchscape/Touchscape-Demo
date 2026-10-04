using UnityEngine;

public class HandMover : MonoBehaviour
{
    private Rigidbody rb;

    public float moveSpeed = 0.1f; // Speed of movement

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        // Moves the hand based on keyboard input
        if (Input.GetKey(KeyCode.UpArrow))
        {
            rb.position += Vector3.up * Time.deltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            rb.position += Vector3.down * Time.deltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            rb.position += Vector3.left * Time.deltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            rb.position += Vector3.right * Time.deltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.W))
        {
            rb.position += Vector3.forward * Time.deltaTime * moveSpeed;
        }
        if (Input.GetKey(KeyCode.S))
        {
            rb.position += Vector3.back * Time.deltaTime * moveSpeed;
        }
    }
}
