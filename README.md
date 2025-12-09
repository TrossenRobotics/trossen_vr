# Trossen VR Teleoperation SDK

A C++ and Python framework that provides interfaces for controlling Trossen Robotics arms using VR controllers via WebSocket communication. The SDK operates in server mode, listening for incoming connections from VR rigs and providing abstractions for handling controller poses and button presses.

---

## Table of Contents

1. [Features](#features)
2. [Requirements](#requirements)
3. [Installation](#installation)
4. [Building the Project](#building-the-project)
5. [Running C++ Demos](#running-c-demos)
6. [Running Python Demos](#running-python-demos)
7. [Example Demos](#example-demos)
8. [Architecture](#architecture)

---

## Features

- **VR Teleoperation**: Control robot arms using VR controller poses and buttons
- **WebSocket Server**: Listens for incoming connections from VR clients
- **Dual API**: Native C++ SDK and Python bindings via pybind11
- **Button Handling**: Support for boolean buttons (A, B, X, Y) and analog inputs (triggers, grips)
- **Pose Tracking**: 6-DOF pose tracking for left and right controllers
- **Manual & Event-Driven Modes**: Poll frames manually or use callback-based teleop handlers

---

## Requirements

### System Dependencies

**C++ Build Tools:**
- CMake >= 3.15
- C++17 compatible compiler (GCC 7+, Clang 5+)
- Make build system

**Required C++ Libraries:**
- [Eigen3](https://eigen.tuxfamily.org/) >= 3.3 (linear algebra)
- [nlohmann/json](https://github.com/nlohmann/json) (JSON parsing)
- [websocketpp](https://github.com/zaphoyd/websocketpp) (WebSocket server)
- [Boost.Asio](https://www.boost.org/) (async I/O, required by websocketpp)
- pthread (threading support)
- [libtrossen_arm](https://github.com/TrossenRobotics/trossen_arm) (Trossen robot driver)

**Python (for Python bindings):**
- Python 3.11 or newer
- [pybind11](https://github.com/pybind/pybind11) >= 2.10
- NumPy (for array operations in Python scripts)
- scipy (for rotation utilities)

### Ubuntu Installation

```bash
# Install system dependencies
sudo apt update
sudo apt install -y cmake build-essential libeigen3-dev nlohmann-json3-dev \
    libboost-all-dev libwebsocketpp-dev python3-dev python3-pip

# Install pybind11 for Python bindings
sudo apt install -y pybind11-dev

# Install Python packages
pip install numpy scipy

# Install trossen_arm python package from PyPI  
pip install trossen_arm
```

---

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/TrossenRobotics/trossen_vr.git
```

### 2. Ensure libtrossen_arm is Installed

The project requires `libtrossen_arm` to be installed and available to CMake. Follow the installation instructions at:

https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html#c

---

## Building the Project

### Build C++ SDK and Python Bindings

```bash
# Create and enter build directory
mkdir -p build
cd ~/trossen_vr

# Create and enter build directory
mkdir -p build
# Configure with CMake
cmake ..

# Build all targets (C++ library, Python module, and example executables)
sudo make install
```

This generates:
- **Static library**: `libtrossen_vr.a` (C++ SDK)
- **Python module**: `pytrossen_vr.cpython-312-x86_64-linux-gnu.so` (or similar, version-dependent)
- **C++ executables**: `event_driven_teleop`, `manual_polling_teleop` (example demos in `demos/cpp/`)

### Build Options

**Build example executables** (optional):
```bash
cmake -DBUILD_DEMOS=ON ..
make
```

---

## Running C++ Demos

The C++ example demos are located in `demos/cpp/` and compiled as executables in the `build/` directory.

### event_driven_teleop (Event-Driven Mode)

Uses callback-based handlers for VR events:

```bash
cd build
./event_driven_teleop
```

**Features:**
- Registers button handlers for A, B, and right trigger
- Uses pose handler for right controller teleoperation
- Robot returns to idle pose on button B press

### manual_polling_teleop (Manual Polling Mode)

Manually polls VR frames in a loop:

```bash
cd build
./manual_polling_teleop
```

**Features:**
- Calls `poll_manual()` to read VR frames directly
- Implements rising-edge button detection
- Handles pause/resume with button A
- Exits and returns to idle with button B

---

## Running Python Demos

### Install Python Module

To build and install the Python module as a wheel:

```bash
# From the project root directory
# First, install the build tool if not already installed
pip install build

# Build the wheel
python -m build

# Install the wheel
pip install dist/trossen_vr-*.whl
```

For development installation (editable mode):

```bash
pip install -e .
```

### Verify Installation

```bash
python3 -c "import pytrossen_vr; print('Module loaded successfully')"
```

### Run Python Examples

Navigate to the Python demos directory:

```bash
cd ../demos/python
```

#### event_driven_teleop.py (Event-Driven Mode)

```bash
python3 event_driven_teleop.py
```

**Features:**
- Uses `poll_teleop()` with registered handlers
- Button A: Toggle pause/resume
- Button B: Return to idle pose
- Right trigger: Control gripper (analog value)
- Right controller pose: Robot teleoperation

#### manual_polling_teleop.py (Manual Polling Mode)

```bash
python3 manual_polling_teleop.py
```

**Features:**
- Uses `poll_manual()` for direct frame access
- Rising-edge button detection
- Button A: Toggle pause/resume
- Button B: Return to idle and exit
- Manual gripper control via trigger values

---

## Example Demos

### C++ Example: Event-Driven Teleoperation

```cpp
#include <trossen_vr/vr_manager.hpp>
#include <trossen_vr/teleop.hpp>

int main() {
    trossen_vr::VRManager::Config cfg;
    cfg.server_port = 5432;
    trossen_vr::VRManager vr_manager(cfg);
    vr_manager.start();

    trossen_vr::Teleop teleop;
    teleop.set_button_A_handler([&]() {
        std::cout << "Button A pressed\n";
    });
    
    teleop.set_right_pose_handler([&](const trossen_vr::VRPose& pose) {
        std::cout << "Right controller at: " 
                  << pose.position[0] << ", " 
                  << pose.position[1] << ", " 
                  << pose.position[2] << "\n";
    });

    while (true) {
        vr_manager.poll_teleop(teleop);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
```

### Python Example: Manual Polling

```python
import pytrossen_vr
import time

# Setup VR Manager
config = pytrossen_vr.VRManagerConfig()
config.server_port = 5432
vr_manager = pytrossen_vr.VRManager(config)
vr_manager.start()

# Polling loop
while True:
    frame = vr_manager.poll_manual()
    if frame is not None:
        # Access button states
        buttons = frame.buttons
        if "a" in buttons and buttons["a"]:
            print("Button A pressed")
        
        # Access pose data
        if frame.right_pose is not None:
            pose = frame.right_pose
            print(f"Right controller: {pose.position}")
    
    time.sleep(0.005)
```

---

## Architecture

### VRManager

The core component that manages WebSocket server communication:

- **Server Mode**: Listens on a configurable port (default: 4582) for VR client connections
- **Frame Reception**: Receives JSON-encoded `VRState` frames containing poses and button states
- **Thread-Safe**: All operations are thread-safe for multi-threaded applications
- **Two Usage Modes**:
  - **Background I/O Thread**: Call `start()` to run continuous frame reception
  - **Manual Polling**: Call `poll_manual()` to read frames on-demand

### Teleop

Event-driven abstraction for handling VR input:

- **Button Handlers**: Register callbacks for discrete button events (A, B, X, Y, triggers, grips)
- **Pose Handlers**: Register callbacks for controller pose updates
- **Rising-Edge Detection**: Automatically handles button state transitions for clean event triggering

### VRState

Represents a complete frame of VR input data:

- `left_pose`, `right_pose`: Optional 6-DOF poses with:
  - `position`: 3D position vector in meters [x, y, z]
  - `rotation`: 3D rotation vector in radians using axis-angle representation where the vector direction is the rotation axis and the magnitude is the rotation angle
- `buttons`: Map of button names to values (variant of `bool` or `double` for analog inputs)
- `timestamp`: Frame timestamp
- `sequence`: Frame sequence number

### Python Bindings

All C++ classes and methods are exposed to Python via pybind11:

- `VRManagerConfig`: Configuration for VRManager
- `VRManager`: Main manager class
- `VRPose`: 6-DOF pose structure
- `VRState`: Complete input frame
- `Teleop`: Event-driven handler registration

---

## Notes

- **Network Configuration**: Ensure the VR client is configured to connect to the server's IP address and port (default 5432)
- **Robot IP**: Update the robot IP address in example scripts (default: `192.168.1.2`)
- **Thread Safety**: All VRManager public methods are thread-safe and can be called from multiple threads


