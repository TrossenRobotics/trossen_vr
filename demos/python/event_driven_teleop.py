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
RIGHT_ARM_IP = "192.168.1.2"
LEFT_ARM_IP = "192.168.1.3"
SEND_RATE_HZ = 100.0
GRIPPER_MAX_M = 0.04
CMD_GOAL_TIME = 0.15

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

# Network
net_config = vr.ReceiverConfig()
net_config.port = 9000
receiver = vr.NetworkManager(net_config)
receiver.start()

# Teleop state
engaged = False
right_offset = np.eye(4)
left_offset = np.eye(4)
right_vr_vec6 = None
left_vr_vec6 = None

teleop = vr.Teleop()


def handle_a():
    global engaged, right_offset, left_offset
    if not engaged:
        if right_vr_vec6 is not None:
            rp = np.array(right_driver.get_cartesian_positions())
            right_offset = vr.vec6_to_T(rp) @ np.linalg.inv(vr.vec6_to_T(right_vr_vec6))
        if left_vr_vec6 is not None:
            lp = np.array(left_driver.get_cartesian_positions())
            left_offset = vr.vec6_to_T(lp) @ np.linalg.inv(vr.vec6_to_T(left_vr_vec6))
        engaged = True
        print("Teleop ENGAGED")
    else:
        engaged = False
        print("Teleop PAUSED")


def handle_b():
    global running
    print("Exit requested via B button")
    running = False


def handle_right_trigger(val):
    right_driver.set_gripper_position(val * GRIPPER_MAX_M, 0.0, False)


def handle_left_trigger(val):
    left_driver.set_gripper_position(val * GRIPPER_MAX_M, 0.0, False)


def handle_right_pose(pose):
    global right_vr_vec6
    right_vr_vec6 = vr.unity_pose_to_vec6(pose.position, pose.rotation)


def handle_left_pose(pose):
    global left_vr_vec6
    left_vr_vec6 = vr.unity_pose_to_vec6(pose.position, pose.rotation)


teleop.on_button(vr.ButtonNames.A, handle_a)
teleop.on_button(vr.ButtonNames.B, handle_b)
teleop.on_analog(vr.ButtonNames.RightTrigger, handle_right_trigger)
teleop.on_analog(vr.ButtonNames.LeftTrigger, handle_left_trigger)
teleop.on_right_pose(handle_right_pose)
teleop.on_left_pose(handle_left_pose)

print(
    "Waiting for VR data... Press A to engage (Press A again to pause), Press B to exit"
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

    teleop.dispatch(frame)

    now = time.monotonic()
    if now - last_send >= send_period and engaged:
        last_send = now
        if right_vr_vec6 is not None:
            T_cmd = right_offset @ vr.vec6_to_T(right_vr_vec6)
            right_driver.set_cartesian_positions(
                vr.T_to_vec6(T_cmd).tolist(),
                trossen_arm.InterpolationSpace.cartesian,
                CMD_GOAL_TIME,
                False,
            )
        if left_vr_vec6 is not None:
            T_cmd = left_offset @ vr.vec6_to_T(left_vr_vec6)
            left_driver.set_cartesian_positions(
                vr.T_to_vec6(T_cmd).tolist(),
                trossen_arm.InterpolationSpace.cartesian,
                CMD_GOAL_TIME,
                False,
            )

receiver.stop()
print("\nShutting down...")
right_driver.set_arm_positions(IDLE_POSE, 2.0, True)
left_driver.set_arm_positions(IDLE_POSE, 2.0, True)
