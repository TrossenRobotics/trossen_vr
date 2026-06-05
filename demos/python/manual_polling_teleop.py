import signal
import time

import numpy as np
import trossen_arm
import trossen_vr as vr

START_POSE = [0, np.pi / 3, np.pi / 6, np.pi / 5, 0, 0]
IDLE_POSE = [0, 0, 0, 0, 0, 0]

running = True


def signal_handler(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Configuration
RIGHT_ARM_IP = "192.168.1.4"
LEFT_ARM_IP = "192.168.1.5"
SEND_RATE_HZ = 100.0
GRIPPER_MAX_M = 0.04
CMD_GOAL_TIME = 0.15

# Robot setup
right_driver = trossen_arm.TrossenArmDriver()
right_driver.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_leader,
    RIGHT_ARM_IP,
    False,
)
right_driver.set_all_modes(trossen_arm.Mode.position)

left_driver = trossen_arm.TrossenArmDriver()
left_driver.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_leader,
    LEFT_ARM_IP,
    False,
)
left_driver.set_all_modes(trossen_arm.Mode.position)

print("Moving arms to start position")
right_driver.set_arm_positions(START_POSE, 2.0, True)
left_driver.set_arm_positions(START_POSE, 2.0, True)

# Network setup
net_config = vr.ReceiverConfig()
net_config.port = 9000
receiver = vr.NetworkManager(net_config)
receiver.start()

# Teleop state
T_offset_right = vr.Transform4D()
T_offset_left = vr.Transform4D()
offset_captured = False

# Track previous tracking state for engage/disengage detection
prev_right_tracked = 0
prev_left_tracked = 0

# Button state tracking for edge detection
prev_button_b = 0

print(
    "Waiting for VR data... Hand/Grip trigger to engage, release to pause. Press B to exit"
)

send_period = 1.0 / SEND_RATE_HZ
last_send = time.monotonic()
last_status = vr.ConnectionStatus.Disconnected

while running:
    # Monitor connection status
    current_status = receiver.get_connection_status()
    if current_status != last_status:
        if current_status == vr.ConnectionStatus.Connecting:
            print("Connecting...")
        elif current_status == vr.ConnectionStatus.Connected:
            print(f"Connection established ({receiver.get_message_frequency():.1f} Hz)")
        elif current_status == vr.ConnectionStatus.Degraded:
            print(
                f"Connection degraded (low frequency: {receiver.get_message_frequency():.1f} Hz)"
            )
        elif current_status == vr.ConnectionStatus.Disconnected:
            print("Connection lost (timeout)")
        last_status = current_status

    frame = receiver.latest_frame()
    if frame is None:
        time.sleep(0.001)
        continue

    # Button B - exit (rising edge detection)
    button_b_current = frame.right_controller.buttons.two
    if button_b_current and not prev_button_b:
        print("Exit requested via B button")
        break
    prev_button_b = button_b_current

    # Tracking state
    right_tracked = frame.right_controller.is_tracked != 0
    left_tracked = frame.left_controller.is_tracked != 0

    # Detect engage (tracking transition 0→1)
    if right_tracked and not prev_right_tracked:
        print("Right controller ENGAGED")
        offset_captured = False
    if left_tracked and not prev_left_tracked:
        print("Left controller ENGAGED")
        offset_captured = False

    # Detect release (tracking transition 1→0)
    if not right_tracked and prev_right_tracked:
        print("Right controller PAUSED")
    if not left_tracked and prev_left_tracked:
        print("Left controller PAUSED")

    prev_right_tracked = right_tracked
    prev_left_tracked = left_tracked

    # Update grippers
    right_driver.set_gripper_position(
        frame.right_controller.triggers.index_trigger * GRIPPER_MAX_M, 0.0, False
    )
    left_driver.set_gripper_position(
        frame.left_controller.triggers.index_trigger * GRIPPER_MAX_M, 0.0, False
    )

    # Capture offset on first valid frame after engage
    if not offset_captured and (right_tracked or left_tracked):
        if right_tracked:
            rp = right_driver.get_cartesian_positions()
            robot_pose_right = vr.Pose6D()
            robot_pose_right.x, robot_pose_right.y, robot_pose_right.z = (
                rp[0],
                rp[1],
                rp[2],
            )
            robot_pose_right.ax, robot_pose_right.ay, robot_pose_right.az = (
                rp[3],
                rp[4],
                rp[5],
            )

            T_robot_right = vr.pose6d_to_transform4d(robot_pose_right)
            T_vr_right = vr.pose6d_to_transform4d(frame.right_controller.pose6d)
            T_offset_right = T_robot_right * T_vr_right.inverse()

        if left_tracked:
            lp = left_driver.get_cartesian_positions()
            robot_pose_left = vr.Pose6D()
            robot_pose_left.x, robot_pose_left.y, robot_pose_left.z = (
                lp[0],
                lp[1],
                lp[2],
            )
            robot_pose_left.ax, robot_pose_left.ay, robot_pose_left.az = (
                lp[3],
                lp[4],
                lp[5],
            )

            T_robot_left = vr.pose6d_to_transform4d(robot_pose_left)
            T_vr_left = vr.pose6d_to_transform4d(frame.left_controller.pose6d)
            T_offset_left = T_robot_left * T_vr_left.inverse()

        offset_captured = True
        continue

    # Rate limiting
    now = time.monotonic()
    if now - last_send < send_period:
        # Sleep for remaining time
        remaining = send_period - (now - last_send)
        time.sleep(remaining)
        continue
    last_send = now

    # Send commands only when controllers are tracked
    if offset_captured and right_tracked:
        T_vr_right = vr.pose6d_to_transform4d(frame.right_controller.pose6d)
        T_cmd_right = T_offset_right * T_vr_right
        cmd_right = vr.transform4d_to_pose6d(T_cmd_right)
        goal = [
            cmd_right.x,
            cmd_right.y,
            cmd_right.z,
            cmd_right.ax,
            cmd_right.ay,
            cmd_right.az,
        ]
        right_driver.set_cartesian_positions(
            goal,
            trossen_arm.InterpolationSpace.cartesian,
            CMD_GOAL_TIME,
            False,
        )

    if offset_captured and left_tracked:
        T_vr_left = vr.pose6d_to_transform4d(frame.left_controller.pose6d)
        T_cmd_left = T_offset_left * T_vr_left
        cmd_left = vr.transform4d_to_pose6d(T_cmd_left)
        goal = [
            cmd_left.x,
            cmd_left.y,
            cmd_left.z,
            cmd_left.ax,
            cmd_left.ay,
            cmd_left.az,
        ]
        left_driver.set_cartesian_positions(
            goal,
            trossen_arm.InterpolationSpace.cartesian,
            CMD_GOAL_TIME,
            False,
        )

receiver.stop()
print("\nShutting down...")
print("Moving arms to idle position")
right_driver.set_arm_positions(IDLE_POSE, 2.0, True)
left_driver.set_arm_positions(IDLE_POSE, 2.0, True)
