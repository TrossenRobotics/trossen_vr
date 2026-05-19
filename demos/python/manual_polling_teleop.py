import time
import signal
import numpy as np
import trossen_vr as vr
import trossen_arm

START_POSE = [0, np.pi/3, np.pi/6, np.pi/5, 0, 0]
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
    config.right_arm_ip, False
)
right_driver.set_all_modes(trossen_arm.Mode.position)

left_driver = trossen_arm.TrossenArmDriver()
left_driver.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_leader,
    config.left_arm_ip, False
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
teleop_active = False
T_offset_right = np.eye(4)
T_offset_left = np.eye(4)
offset_captured = False
prev_buttons = {}

print("Waiting for VR data... Press A to start, B to exit.")

send_period = 1.0 / config.send_rate_hz
last_send = time.monotonic()


def button_pressed(buttons, name):
    val = buttons.get(name)
    if val is None or not isinstance(val, bool):
        return False
    previous = prev_buttons.get(name, False)
    prev_buttons[name] = val
    return val and not previous


while running:
    frame = receiver.latest_frame()
    if frame is None:
        time.sleep(0.001)
        continue

    # Button handling
    if button_pressed(frame.buttons, "a"):
        teleop_active = not teleop_active
        offset_captured = False
        print("Teleop ENGAGED" if teleop_active else "Teleop PAUSED")

    if button_pressed(frame.buttons, "b"):
        print("Exit requested via B button")
        break

    # Grippers
    right_trig = frame.buttons.get("rightTrigger", 0.0)
    if isinstance(right_trig, float):
        right_driver.set_gripper_position(right_trig * config.gripper_max_m, 0.0, False)

    left_trig = frame.buttons.get("leftTrigger", 0.0)
    if isinstance(left_trig, float):
        left_driver.set_gripper_position(left_trig * config.gripper_max_m, 0.0, False)

    if not teleop_active:
        continue

    # Poses
    right_v6 = None
    left_v6 = None

    if frame.right is not None:
        right_v6 = np.array(vr.unity_pose_to_vec6(frame.right.position, frame.right.rotation))
    if frame.left is not None:
        left_v6 = np.array(vr.unity_pose_to_vec6(frame.left.position, frame.left.rotation))

    # Capture offset
    if not offset_captured:
        if right_v6 is not None:
            rp = np.array(right_driver.get_cartesian_positions())
            T_offset_right = vr.vec6_to_T(rp) @ np.linalg.inv(vr.vec6_to_T(right_v6))
        if left_v6 is not None:
            lp = np.array(left_driver.get_cartesian_positions())
            T_offset_left = vr.vec6_to_T(lp) @ np.linalg.inv(vr.vec6_to_T(left_v6))
        if right_v6 is not None or left_v6 is not None:
            offset_captured = True
            print("Offset captured — tracking active")
        continue

    # Rate-limited commands
    now = time.monotonic()
    if now - last_send < send_period:
        continue
    last_send = now

    if right_v6 is not None:
        T_cmd = T_offset_right @ vr.vec6_to_T(right_v6)
        right_driver.set_cartesian_positions(
            vr.T_to_vec6(T_cmd).tolist(),
            trossen_arm.InterpolationSpace.cartesian,
            config.cmd_goal_time, False
        )

    if left_v6 is not None:
        T_cmd = T_offset_left @ vr.vec6_to_T(left_v6)
        left_driver.set_cartesian_positions(
            vr.T_to_vec6(T_cmd).tolist(),
            trossen_arm.InterpolationSpace.cartesian,
            config.cmd_goal_time, False
        )

receiver.stop()
print("\nShutting down...")
right_driver.set_arm_positions(IDLE_POSE, 2.0, True)
left_driver.set_arm_positions(IDLE_POSE, 2.0, True)
