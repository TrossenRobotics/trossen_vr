import asyncio
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import websockets
import trossen_arm
import time

# ---------------------------
# Robot Pose Definitions
# ---------------------------
START_POSE = [0, np.pi/12, np.pi/12, 0, 0, 0]
IDLE_POSE  = [0, 0, 0, 0, 0, 0]

# ---------------------------
# Driver Setup
# ---------------------------
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

cartesian_positions = driver.get_cartesian_positions()
print("Initial Cartesian Positions:", cartesian_positions)

# ---------------------------
# Helper Functions
# ---------------------------
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


# ---------------------------
# Connection Handler
# ---------------------------
async def handler(websocket):
    print("WebSocket client connected.")
    
    # Move robot to start pose on new connection
    driver.set_arm_positions(START_POSE, goal_time=3.0, blocking=True)
    print("Robot moved to START pose for teleop session.")

    init_robot_pose = None
    init_controller_pose = None
    T_offset = None
    last_send_time = 0
    send_rate_hz = 200.0
    send_period = 1.0 / send_rate_hz

    try:
        async for message in websocket:
            data = json.loads(message)

            # Handle exit command (no socket close)
            if "command" in data and data["command"] == "exit":
                print("Exit command received. Moving robot to IDLE pose...")
                driver.set_arm_positions(IDLE_POSE, goal_time=3.0, blocking=True)
                print("Robot is idle. Waiting for next teleop command or reconnect.")
                continue

            # Handle pose data
            pose = np.array(data.get("pose"), dtype=float)

            # # Initialize transforms
            # if init_controller_pose is None:
            #     init_controller_pose = pose
            #     init_robot_pose = np.array(cartesian_positions)
            #     print("Captured initial controller and robot poses.")
            #     T_robot_start = vec6_to_T(init_robot_pose)
            #     T_quest_start = vec6_to_T(init_controller_pose)
            #     T_offset = T_robot_start @ np.linalg.inv(T_quest_start)
            #     continue
            
            if "teleopEnabled" in data and data["teleopEnabled"]:
                # Reset offsets and initial mapping every time teleop is re-enabled or starts initially
                cartesian_positions = driver.get_cartesian_positions()
                init_robot_pose = np.array(cartesian_positions)
                init_controller_pose = np.array(data.get("pose"), dtype=float)
                T_robot_start = vec6_to_T(init_robot_pose)
                T_quest_start = vec6_to_T(init_controller_pose)
                T_offset = T_robot_start @ np.linalg.inv(T_quest_start)
                print("Teleop resumed — recalibrated controller alignment.")

            # send fixed rate of commands to robot (200 Hz)
            now = time.time()
            if now - last_send_time < send_period:
                continue
            last_send_time = now

            # Apply transform from controller to robot
            Tq = vec6_to_T(pose)
            Tt = T_offset @ Tq
            robot_cmd_vec = T_to_vec6(Tt)

            # Send position command
            driver.set_cartesian_positions(
                robot_cmd_vec,
                trossen_arm.InterpolationSpace.cartesian,
                goal_time=0.15,
                blocking=False,
            )

            # Gripper control
            gripper_value = float(data.get("gripperValue", 0.0))
            position = gripper_value * 0.04
            driver.set_gripper_position(position, 0.0, False)

    except websockets.ConnectionClosed:
        print("Client disconnected — returning to IDLE pose.")
        driver.set_arm_positions(IDLE_POSE, goal_time=3.0, blocking=True)
        print("Robot returned to IDLE pose and awaiting next connection.")


async def main():
    async with websockets.serve(handler, "0.0.0.0", 5432, ping_interval=None):
        print("Teleop server running on ws://0.0.0.0:5432")
        await asyncio.Future()  # run forever

asyncio.run(main())
