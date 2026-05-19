using UnityEngine;

// data container class.
[System.Serializable]
public class TeleopData
{
    // Vector3 x, y, z position
    public Vector3 rightPosition;
    public Vector3 leftPosition;

    // Quaternion x, y, z, w rotation
    public Quaternion rightRotation;
    public Quaternion leftRotation;

    // Generic button map — all inputs in one place
    public ButtonState buttons;
}

[System.Serializable]
public class ButtonState
{
    // Analog (0.0 – 1.0)
    public float rightTrigger;
    public float leftTrigger;
    public float rightGrip;
    public float leftGrip;

    // Digital (true/false) — Right controller: A, B | Left controller: X, Y
    public bool a;
    public bool b;
    public bool x;
    public bool y;
}
