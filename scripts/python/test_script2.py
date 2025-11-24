import time
import math
import numpy as np

import pytrossen_vr
import trossen_arm

# ---------------------------
# Utils
# ---------------------------
def vec6_to_T(v6):
    p = np.array(v6[:3])
    rvec = np.array(v6[3:])
    angle = np.linalg.norm(rvec)
    R = np.eye(3)
    if angle > 1e-8:
        axis = rvec / angle
        R = rotation_matrix_from_axis_angle(axis, angle)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def T_to_vec6(T):
    p = T[:3, 3]
    R = T[:3, :3]
    rvec = axis_angle_from_rotation_matrix(R)
    return [p[0], p[1], p[2], rvec[0], rvec[1], rvec[2]]


def pose_to_vec6(pose):
    if pose is None:
        return None
    return [
        pose.position[0],
        pose.position[1],
        pose.position[2],
        pose.rotation[0],
        pose.rotation[1],
        pose.rotation[2]
    ]


def rotation_matrix_from_axis_angle(axis, angle):
    # Rodrigues formula
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def axis_angle_from_rotation_matrix(R):
    angle = math.acos((np.trace(R) - 1) / 2)
    if abs(angle) < 1e-8:
        return np.zeros(3)
    rx = (R[2, 1] - R[1, 2]) / (2 * math.sin(angle))
    ry = (R[0, 2] - R[2, 0]) / (2 * math.sin(angle))
    rz = (R[1, 0] - R[0, 1]) / (2 * math.sin(angle))
    return np.array([rx, ry, rz]) * angle


# ---------------------------
# Robot setup
# ---------------------------
START_POSE = [0, math.pi/12, math.pi/12, 0, 0, 0]
IDLE_POSE  = [0, 0, 0, 0, 0, 0]

driver = trossen_arm.TrossenArmDriver()
driver.configure(trossen_arm.Model.wxai_v0,
                 trossen_arm.StandardEndEffector.wxai_v0_follower,
                 "192.168.1.2", True)
driver.set_arm_modes(trossen_arm.Mode.position)
driver.set_gripper_mode(trossen_arm.Mode.position)
driver.set_arm_positions(START_POSE, 3.0, True)
print("Robot ready.")


# ---------------------------
# VRManager setup (manual mode)
# ---------------------------
cfg = pytrossen_vr.VRManagerConfig()
cfg.server_port = 5432
vr_manager = pytrossen_vr.VRManager(cfg)
vr_manager.start()
print("VR Manager started in MANUAL mode.")


# ---------------------------
# Teleop state
# ---------------------------
pause_teleop = False
init_right_pose = None
init_robot_pose = None
T_offset_right = np.eye(4)

# Previous button states for rising edge detection
prev_buttons = {}

def button_pressed(frame, name):
    val = frame.buttons.get(name)
    if val is None or not isinstance(val, bool):
        return False
    current = val
    previous = prev_buttons.get(name, False)
    prev_buttons[name] = current
    return (not previous) and current


# ---------------------------
# Main loop (~200 Hz)
# ---------------------------
try:
    while True:
        frame = vr_manager.poll_manual()
        if frame is None:
            continue

        # ---------------------
        # BUTTON HANDLING
        # ---------------------
        if button_pressed(frame, "a"):
            pause_teleop = not pause_teleop
            print("Teleop PAUSED" if pause_teleop else "Teleop RESUMED")

        if button_pressed(frame, "b"):
            print("Returning to IDLE pose.")
            driver.set_arm_positions(IDLE_POSE, 3.0, True)
            break

        # ---------------------
        # GRIPPER TRIGGER
        # ---------------------
        trig_val = frame.buttons.get("right_trigger")
        if isinstance(trig_val, float):
            driver.set_gripper_position(trig_val * 0.04, 0.0, False)

        # ---------------------
        # RIGHT CONTROLLER (main teleop)
        # ---------------------
        if not pause_teleop:
            right_v6 = pose_to_vec6(frame.right_pose)
            if right_v6 is not None:
                if init_right_pose is None:
                    init_robot_pose = driver.get_cartesian_positions()
                    init_right_pose = right_v6
                    T_offset_right = vec6_to_T(init_robot_pose) @ np.linalg.inv(vec6_to_T(init_right_pose))
                    print("Teleop initialized.")

                Tq = vec6_to_T(right_v6)
                Tt = T_offset_right @ Tq
                robot_cmd = T_to_vec6(Tt)
                driver.set_cartesian_positions(robot_cmd,
                                               trossen_arm.InterpolationSpace.cartesian,
                                               0.15, False)

        time.sleep(0.005)

finally:
    driver.set_arm_positions(IDLE_POSE, 3.0, True)
