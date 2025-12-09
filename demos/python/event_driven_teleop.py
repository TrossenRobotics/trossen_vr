import asyncio
import numpy as np
from scipy.spatial.transform import Rotation as R
import pytrossen_vr
import trossen_arm

START_POSE = [0, np.pi/12, np.pi/12, 0, 0, 0]
IDLE_POSE  = [0, 0, 0, 0, 0, 0]

driver = trossen_arm.TrossenArmDriver()
driver.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_follower,
    "192.168.1.2",
    True
)
driver.set_arm_modes(trossen_arm.Mode.position)
driver.set_arm_positions(IDLE_POSE, goal_time=3.0, blocking=True)
driver.set_gripper_mode(trossen_arm.Mode.position)
driver.set_arm_positions(START_POSE, goal_time=3.0, blocking=True)
print("Robot ready for teleop.")

def vec6_to_T(v6):
    p = np.asarray(v6[:3], dtype=float)
    rvec = np.asarray(v6[3:], dtype=float)
    Rm = R.from_rotvec(rvec).as_matrix()
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rm, p
    return T

def T_to_vec6(T):
    p = T[:3, 3]
    rvec = R.from_matrix(T[:3, :3]).as_rotvec()
    return np.concatenate([p, rvec])

def parse_vr_pose(pose):
    if pose is None:
        return None
    return np.array(pose.position + pose.rotation, dtype=float)

# VRManager Setup
vr_config = pytrossen_vr.VRManagerConfig()
vr_config.server_port = 4582
vr_manager = pytrossen_vr.VRManager(vr_config)
vr_manager.start()
print("VRManager started, connecting to VR client.")

# Teleop callbacks
pause_telop = True
init_right_pose = None
T_offset_right = None
init_robot_pose = None

def handle_right_pose(pose):
    global pause_telop, init_right_pose, T_offset_right, init_robot_pose
    
    if pause_telop:
        return
    
    right_pose_vec = parse_vr_pose(pose)
    if right_pose_vec is None:
        return

    if init_right_pose is None:
        cartesian_positions = driver.get_cartesian_positions()
        init_right_pose = right_pose_vec
        init_robot_pose = np.array(cartesian_positions)
        T_robot_start = vec6_to_T(init_robot_pose)
        T_right_start = vec6_to_T(init_right_pose)
        T_offset_right = T_robot_start @ np.linalg.inv(T_right_start)
        print("Teleop initialized — right controller aligned.")

    # Map right controller to robot
    Tq = vec6_to_T(right_pose_vec)
    Tt = T_offset_right @ Tq
    robot_cmd_vec = T_to_vec6(Tt)

    driver.set_cartesian_positions(
        robot_cmd_vec,
        trossen_arm.InterpolationSpace.cartesian,
        goal_time=0.15,
        blocking=False
    )

def handle_button_a():
    global pause_telop
    pause_telop = not pause_telop
    print(f"Teleop {'paused' if pause_telop else 'resumed'}")

def handle_button_b():
    driver.set_arm_positions(IDLE_POSE, goal_time=3.0, blocking=True)
    global pause_telop
    pause_telop = not pause_telop

def handle_right_trigger():
    # Get the analog trigger value from VRManager
    trigger_val = vr_manager.get_button_state("right_trigger")
    if trigger_val is not None:
        if isinstance(trigger_val, float):
            driver.set_gripper_position(trigger_val * 0.04, 0.0, False)

# Bind the callbacks
teleop = pytrossen_vr.Teleop()
teleop.set_right_pose_handler(handle_right_pose)
teleop.set_button_A_handler(handle_button_a)
teleop.set_button_B_handler(handle_button_b)
teleop.set_button_right_trigger_handler(handle_right_trigger)

# Polling loop
async def main_loop():

    while True:
        vr_manager.poll_teleop(teleop)
        await asyncio.sleep(0.005)  # ~200 Hz loop
asyncio.run(main_loop())
