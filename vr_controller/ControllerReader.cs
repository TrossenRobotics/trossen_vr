using UnityEngine;

public class ControllerReader : MonoBehaviour
{
    [Header("Network")]
    public NetworkSender networkSender;

    void Update()
    {
        // Read right controller
        Vector3 rightPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);
        Quaternion rightRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.RTouch);
        float rightTrigger = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        float rightGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.RTouch);

        // Read left controller
        Vector3 leftPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch);
        Quaternion leftRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.LTouch);
        float leftTrigger = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
        float leftGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch);

        // Read face buttons (A/B on right, X/Y on left)
        bool buttonA = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.RTouch);
        bool buttonB = OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.RTouch);
        bool buttonX = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.LTouch);
        bool buttonY = OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.LTouch);

        // Pack all data into one TeleopData object
        TeleopData data = new TeleopData();
        data.rightPosition = rightPosition;
        data.rightRotation = rightRotation;
        data.leftPosition = leftPosition;
        data.leftRotation = leftRotation;

        data.buttons = new ButtonState();
        data.buttons.rightTrigger = rightTrigger;
        data.buttons.leftTrigger = leftTrigger;
        data.buttons.rightGrip = rightGrip;
        data.buttons.leftGrip = leftGrip;
        data.buttons.a = buttonA;
        data.buttons.b = buttonB;
        data.buttons.x = buttonX;
        data.buttons.y = buttonY;

        networkSender.Send(data);
    }
}