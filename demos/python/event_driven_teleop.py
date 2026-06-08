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


# Arm state management
class ArmTeleopState:
    def __init__(self):
        self.engaged = False
        self.T_offset = vr.Transform4D()
        self.last_vr_pose = vr.Pose6D()
        self.pose_valid = False


right_state = ArmTeleopState()
left_state = ArmTeleopState()


def engage_arm(state, driver):
    if state.engaged or not state.pose_valid:
        return

    current_p = driver.get_cartesian_positions()
    robot_pose = vr.Pose6D()
    robot_pose.x, robot_pose.y, robot_pose.z = current_p[0], current_p[1], current_p[2]
    robot_pose.ax, robot_pose.ay, robot_pose.az = (
        current_p[3],
        current_p[4],
        current_p[5],
    )

    T_robot = vr.pose6d_to_transform4d(robot_pose)
    T_vr = vr.pose6d_to_transform4d(state.last_vr_pose)
    state.T_offset = T_robot * T_vr.inverse()
    state.engaged = True


def disengage_arm(state):
    if not state.engaged:
        return
    state.engaged = False
    state.pose_valid = False


def send_arm_command(state, driver, cmd_goal_time):
    if not state.engaged or not state.pose_valid:
        return

    T_vr = vr.pose6d_to_transform4d(state.last_vr_pose)
    T_cmd = state.T_offset * T_vr
    cmd_pose = vr.transform4d_to_pose6d(T_cmd)
    goal = [cmd_pose.x, cmd_pose.y, cmd_pose.z, cmd_pose.ax, cmd_pose.ay, cmd_pose.az]
    driver.set_cartesian_positions(
        goal, trossen_arm.InterpolationSpace.cartesian, cmd_goal_time, False
    )


# Track previous tracking state for automatic engage/disengage
prev_right_tracked = 0
prev_left_tracked = 0

# Event-driven teleop setup
teleop = vr.Teleop()


# Button B - exit
def handle_b():
    global running
    print("Exit requested via B button")
    running = False


# Right trigger - right gripper
def handle_right_trigger(val):
    right_driver.set_gripper_position(val * GRIPPER_MAX_M, 0.0, False)


# Left trigger - left gripper
def handle_left_trigger(val):
    left_driver.set_gripper_position(val * GRIPPER_MAX_M, 0.0, False)


# Right controller pose - auto engage/disengage based on tracking
def handle_right_pose(pose):
    global prev_right_tracked
    right_state.last_vr_pose = pose
    right_state.pose_valid = True

    if not prev_right_tracked:
        # Auto-engage arm
        engage_arm(right_state, right_driver)
        print("Right arm ENGAGED")
    prev_right_tracked = 1


# Left controller pose - auto engage/disengage based on tracking
def handle_left_pose(pose):
    global prev_left_tracked
    left_state.last_vr_pose = pose
    left_state.pose_valid = True

    if not prev_left_tracked:
        # Auto-engage arm
        engage_arm(left_state, left_driver)
        print("Left arm ENGAGED")
    prev_left_tracked = 1


teleop.on_button_b(handle_b)
teleop.on_right_trigger(handle_right_trigger)
teleop.on_left_trigger(handle_left_trigger)
teleop.on_right_pose(handle_right_pose)
teleop.on_left_pose(handle_left_pose)

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

    # Dispatch to event handlers
    teleop.dispatch(frame)

    # Check for tracking release - disengage
    right_tracked = frame.right_controller.is_tracked != 0
    left_tracked = frame.left_controller.is_tracked != 0

    if not right_tracked and prev_right_tracked:
        disengage_arm(right_state)
        print("Right arm PAUSED")
        prev_right_tracked = 0
    if not left_tracked and prev_left_tracked:
        disengage_arm(left_state)
        print("Left arm PAUSED")
        prev_left_tracked = 0

    # Rate-limited command sending
    now = time.monotonic()
    if now - last_send >= send_period:
        send_arm_command(right_state, right_driver, CMD_GOAL_TIME)
        send_arm_command(left_state, left_driver, CMD_GOAL_TIME)
        last_send = now

receiver.stop()
print("\nShutting down...")
print("Moving arms to idle position")
right_driver.set_arm_positions(IDLE_POSE, 2.0, True)
left_driver.set_arm_positions(IDLE_POSE, 2.0, True)
