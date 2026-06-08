# Trossen VR Teleoperation

C++ and Python SDK for controlling Trossen robot arms using Meta Quest VR controllers over network communication.

## Requirements

- [nlohmann/json](https://github.com/nlohmann/json) >= 3.2
- [libtrossen_arm](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html#c) (for demos)
- Python >= 3.11 (for Python bindings)
- [uv](https://docs.astral.sh/uv/) (for Python environment management)
- CMake >= 3.15
- C++17 compiler

## Installation

### System Dependencies

```bash
sudo apt install -y cmake build-essential nlohmann-json3-dev
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Follow the [libtrossen_arm C++ installation guide](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html#c) to install the robot driver if you want to run the demos.

### Build C++ Library

```bash
# Configure and build
cmake -B build
cmake --build build -j$(nproc)

# Install library and headers (optional, for use in other projects)
sudo cmake --install build
```

### Build C++ Demos (Optional)

```bash
# Configure with demos enabled (requires libtrossen_arm)
cmake -B build -DBUILD_DEMOS=ON
cmake --build build -j$(nproc)
```

### Build Python Bindings (Optional)

```bash
# Create Python environment
uv venv
uv sync

# Configure with Python bindings enabled
source .venv/bin/activate
cmake -B build -DBUILD_PYTHON=ON

# Build and install Python module
cmake --build build -j$(nproc)
sudo cmake --install build
```

## Usage

**C++**: Include the library with a single header:
```cpp
#include "trossen_vr/trossen_vr.hpp"
```

**Python**: Import the module:
```python
import trossen_vr as vr
```

## Demos

> **Before running any demo**, start the Trossen VR Teleop app on your Meta Quest headset. In the app, enter the IP address of the PC running the demo and press **Connect**. The headset and PC must be on the same network.

> **Update the robot IP addresses** in the demo source files to match your setup. The defaults are `192.168.1.4` (right arm) and `192.168.1.5` (left arm).

### Controls

- **Hand/Grip Trigger**: Hold to enable tracking and engage arm (deadman switch). Release to pause.
- **Index Trigger**: Control gripper open/close.
- **B Button** (right controller): Exit program.

Tracking and engagement happen automatically when you hold the hand trigger. Release the trigger to pause control while keeping the program running.

### C++ Demos

Located in `demos/cpp/`:

- **`event_driven_teleop`**: Event-driven dual-arm teleop using callback handlers. Demonstrates the Teleop API with automatic engage/pause via deadman switch.
- **`manual_polling_teleop`**: Manual frame polling with inline state tracking. Same functionality, different implementation pattern showing direct frame access.

```bash
./build/event_driven_teleop
./build/manual_polling_teleop
```

### Python Demos

Located in `demos/python/`:

- **`event_driven_teleop.py`**: Python version of event-driven demo with callback-based control.
- **`manual_polling_teleop.py`**: Python version of manual polling demo with direct frame access.

```bash
source .venv/bin/activate
python demos/python/event_driven_teleop.py
python demos/python/manual_polling_teleop.py
```
