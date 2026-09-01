using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class LocalAxisTorqueController : MonoBehaviour
{
    public Rigidbody rb; // Reference to the Rigidbody component
    public KeyCode rotateLeftKey = KeyCode.Q; // Default key for rotating left
    public KeyCode rotateRightKey = KeyCode.E; // Default key for rotating right
    public float torqueAmount = 10f; // Amount of torque to apply

    void Awake()
    {
        // Ensure there's a Rigidbody component attached to the GameObject
        if (!rb) rb = GetComponent<Rigidbody>();
    }


    void FixedUpdate()
    {
        // Check for player input and apply torque accordingly
        if (IsKeyPressed(rotateLeftKey))
        {
            ApplyTorque(-torqueAmount);
        }
        else if (IsKeyPressed(rotateRightKey))
        {
            ApplyTorque(torqueAmount);
        }
    }

    void ApplyTorque(float amount)
    {
        // Apply torque in the local up axis
        rb.AddTorque(transform.up * amount, ForceMode.Acceleration);
    }

    private bool IsKeyPressed(KeyCode keyCode)
    {
        if (Keyboard.current == null) return false;
        
        Key key = ConvertKeyCodeToKey(keyCode);
        if (key == Key.None) return false;
        
        return Keyboard.current[key].isPressed;
    }

    private Key ConvertKeyCodeToKey(KeyCode keyCode)
    {
        switch (keyCode)
        {
            case KeyCode.Q: return Key.Q;
            case KeyCode.E: return Key.E;
            default: return Key.None;
        }
    }
}
