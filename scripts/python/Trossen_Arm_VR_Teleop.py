import asyncio
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import websockets
import trossen_arm
import time

START_POSE = [0, np.pi/12, np.pi/12, 0, 0, 0]
IDLE_POSE  = [0, 0, 0, 0, 0, 0]

# Driver Setup
driver = trossen_arm.TrossenArmDriver()

print("Configuring drivers...")
driver.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_follower,
    "192.168.1.2",
    True
)
driver.set_arm_modes(trossen_arm.Mode.position)
driver.set_arm_positions(IDLE_POSE, goal_time=3.0, blocking=True)
driver.set_gripper_mode(trossen_arm.Mode.position)

# Move robot to start pose on new connection
driver.set_arm_positions(START_POSE, goal_time=3.0, blocking=True)
print("Robot moved to START pose for teleop session.")

cartesian_positions = driver.get_cartesian_positions()
print("Initial Cartesian Positions:", cartesian_positions)

# Helper Functions
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

def parse_vr_pose(pose_dict):
    if pose_dict is None:
        return None
    return np.array(pose_dict["position"] + pose_dict["rotation"], dtype=float)

# Connection Handler
async def handler(websocket):
    print("WebSocket client connected.")

    init_robot_pose = None
    init_right_pose = None
    T_offset_right = None

    last_send_time = 0
    send_rate_hz = 200.0
    send_period = 1.0 / send_rate_hz

    pause_telop = True

    try:
        async for message in websocket:
            data = json.loads(message)
            buttons = data.get("buttons", {})

            # Handle exit command
            if buttons.get("b") == True:
                print("Exit command received. Moving robot to IDLE pose...")
                driver.set_arm_positions(IDLE_POSE, goal_time=3.0, blocking=True)
                continue
            
            if buttons.get("a") == True:
                pause_telop = not pause_telop

            if pause_telop == True:
                if buttons.get("a") == True:
                    print("Telop pause command received.")
                init_right_pose = None
                continue
            else:
                if buttons.get("a") == True:
                    print("Telop resume command received.")

            # Extract right controller pose
            right_pose_vec = parse_vr_pose(data.get("right_pose"))

            # Initialize transform on first teleop frame
            if right_pose_vec is not None and init_right_pose is None:
                cartesian_positions = driver.get_cartesian_positions()
                init_right_pose = right_pose_vec
                init_robot_pose = np.array(cartesian_positions)
                T_robot_start = vec6_to_T(init_robot_pose)
                T_right_start = vec6_to_T(init_right_pose)
                T_offset_right = T_robot_start @ np.linalg.inv(T_right_start)
                print("Teleop initialized — right controller alignment captured.")

            # Fixed rate send
            now = time.time()
            if now - last_send_time < send_period:
                continue
            last_send_time = now

            # Map right controller to robot
            Tq = vec6_to_T(right_pose_vec)
            Tt = T_offset_right @ Tq
            robot_cmd_vec = T_to_vec6(Tt)

            driver.set_cartesian_positions(
                robot_cmd_vec,
                trossen_arm.InterpolationSpace.cartesian,
                goal_time=0.15,
                blocking=False,
            )

            # Gripper control
            right_trigger_val = buttons.get("right_trigger", 0.0)
            driver.set_gripper_position(right_trigger_val * 0.04, 0.0, False)

    except websockets.ConnectionClosed:
        print("Client disconnected — returning to IDLE pose.")
        driver.set_arm_positions(IDLE_POSE, goal_time=3.0, blocking=True)
        print("Robot returned to IDLE pose.")

# Run WebSocket Server
async def main():
    async with websockets.serve(handler, "0.0.0.0", 5432, ping_interval=None):
        print("Teleop server running on ws://0.0.0.0:5432")
        await asyncio.Future()  # run forever

asyncio.run(main())
