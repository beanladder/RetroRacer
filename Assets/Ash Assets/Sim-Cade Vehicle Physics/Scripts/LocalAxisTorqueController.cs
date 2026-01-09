using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class LocalAxisTorqueController : MonoBehaviour
{
    public Rigidbody rb; // Reference to the Rigidbody component
    public float torqueAmount = 10f; // Amount of torque to apply

    void Awake()
    {
        // Ensure there's a Rigidbody component attached to the GameObject
        if (!rb) rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        // Check for player input using new input system (Q and E keys)
        if (Keyboard.current != null)
        {
            if (Keyboard.current.qKey.isPressed)
            {
                ApplyTorque(-torqueAmount);
            }
            else if (Keyboard.current.eKey.isPressed)
            {
                ApplyTorque(torqueAmount);
            }
        }
    }

    void ApplyTorque(float amount)
    {
        // Apply torque in the local up axis
        rb.AddTorque(transform.up * amount, ForceMode.Acceleration);
    }
}
