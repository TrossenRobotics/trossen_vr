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


config = vr.TeleopConfig()
config.right_arm_ip = "192.168.1.4"
config.left_arm_ip = "192.168.1.2"
config.send_rate_hz = 100.0
config.gripper_max_m = 0.04
config.cmd_goal_time = 0.15

right_driver = trossen_arm.TrossenArmDriver()
right_driver.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_leader,
    config.right_arm_ip,
    False,
)
right_driver.set_all_modes(trossen_arm.Mode.position)

left_driver = trossen_arm.TrossenArmDriver()
left_driver.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_leader,
    config.left_arm_ip,
    False,
)
left_driver.set_all_modes(trossen_arm.Mode.position)

print("Moving arms to start position")
right_driver.set_arm_positions(START_POSE, 2.0, True)
left_driver.set_arm_positions(START_POSE, 2.0, True)

# Network
net_config = vr.ReceiverConfig()
net_config.port = 9000
receiver = vr.UDPReceiver(net_config)
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
    right_driver.set_gripper_position(val * config.gripper_max_m, 0.0, False)


def handle_left_trigger(val):
    left_driver.set_gripper_position(val * config.gripper_max_m, 0.0, False)


def handle_right_pose(pose):
    global right_vr_vec6
    right_vr_vec6 = vr.unity_pose_to_vec6(pose.position, pose.rotation)


def handle_left_pose(pose):
    global left_vr_vec6
    left_vr_vec6 = vr.unity_pose_to_vec6(pose.position, pose.rotation)


teleop.on_button("a", handle_a)
teleop.on_button("b", handle_b)
teleop.on_analog("rightTrigger", handle_right_trigger)
teleop.on_analog("leftTrigger", handle_left_trigger)
teleop.on_right_pose(handle_right_pose)
teleop.on_left_pose(handle_left_pose)

print("Waiting for VR data... Press A to engage teleop.")

send_period = 1.0 / config.send_rate_hz
last_send = time.monotonic()

while running:
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
                config.cmd_goal_time,
                False,
            )
        if left_vr_vec6 is not None:
            T_cmd = left_offset @ vr.vec6_to_T(left_vr_vec6)
            left_driver.set_cartesian_positions(
                vr.T_to_vec6(T_cmd).tolist(),
                trossen_arm.InterpolationSpace.cartesian,
                config.cmd_goal_time,
                False,
            )

receiver.stop()
print("\nShutting down...")
right_driver.set_arm_positions(IDLE_POSE, 2.0, True)
left_driver.set_arm_positions(IDLE_POSE, 2.0, True)
