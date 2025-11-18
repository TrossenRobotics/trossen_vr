# Architecture

## 1. VRManager  
Manages the link to the VR application. 

Responsibilities:

- Opening and closing the WebSocket connection.
- Receiving VR input packets: controller pose, button states, menu commands.
- Sending acknowledgments or outbound messages if needed.
- Handling safe startup and shutdown behavior.
- API for retrieving the latest VR data.

### Conceptual Class Breakdown
**Class: `VRManager`**
- **Fields**
  - `connection`  
  - `is_connected`  
  - `latest_pose`  
  - `latest_button_actions`  

- **Connection Functions**
  - `start()`  
  - `stop()`  
  - `restart()`  
  - `is_active()`  

- **Data Functions**
  - `get_pose()`  
  - `send_updates(Teleop)` 
  - `get_button_state(button_name)`  

---

## 2. Teleop  
Defines a lightweight interface for configuring teleoperation behavior and mapping VR inputs to abstract actions the user can define.

Responsibilities:

- button mappings (e.g., A, B, trigger, grip)
- handler callbacks for start/pause/resume
- pose mapping callbacks
- configuration before running

Your script (outside the SDK) plugs robot logic into these handlers.

### Conceptual Class Breakdown
**Abstract Class: `Teleop`**
- **Fields**
  - `button_map` (handler function)
  - `on_start_handler`
  - `on_pause_handler`
  - `on_resume_handler`
  - `pose_handler`
  - `config_flags`

- **Configuration Functions**
  - `set_button_A(callback)`
  - `set_button_B(callback)`
  - `set_button_trigger(callback)`
  - `set_pose_handler(callback)`
  - `configure(...)`
  
All required handlers should be configured before use (on_start_handler, pose_handler).


## 3. User’s Script Layer (Robot-Specific Code)

This is where the user writes their integration code.  
The SDK does not know any details about the robot or the API interface the robot uses.

### Example structure (conceptual example of how we could use the above in a script)
```python

from trossen_meta_vr import VRManager, Teleop
from my_robot_api import MyRobotDriver

robot = MyRobotDriver()
robot.connect() 

vr = VRManager()
vr.start()


t = Teleop()

def on_start():
    robot.reset_motion_state()
    print("Teleop started.")

t.set_button_A(on_start)

def on_pause():
    robot.stop_motion()
    print("Teleop paused.")

def on_resume():
    robot.realign_to_current_pose()
    print("Teleop resumed.")

t.set_button_B_pause(on_pause)
t.set_button_B_resume(on_resume)

def handle_pose(vr_pose):
    robot_pose_cmd = compute_robot_pose_from_vr(vr_pose)
    robot.set_cartesian_pose(robot_pose_cmd)

t.set_pose_handler(handle_pose)

while True:
    vr_pose = vr.get_pose()
    buttons = vr.latest_button_actions
    
    robot.pose_handler(vr_pose)

    if t.evaluate_button_states(buttons):
        break


vr.stop()
robot.disconnect()
```