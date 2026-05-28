# Trossen VR Teleoperation

C++ and Python SDK for controlling Trossen robot arms using Meta Quest VR controllers over UDP.

## Requirements

- [Eigen3](https://eigen.tuxfamily.org/) >= 3.3
- [nlohmann/json](https://github.com/nlohmann/json) >= 3.2
- [libtrossen_arm](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html#c)
- Python >= 3.11
- [uv](https://docs.astral.sh/uv/)

## Setup

```bash
sudo apt install -y cmake build-essential libeigen3-dev nlohmann-json3-dev
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Follow the [libtrossen_arm C++ installation guide](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html#c) to install the libtrossen_arm robot driver.

## Build

### Python environment

```bash
uv venv
uv sync
```

### C++ and Python bindings

```bash
source .venv/bin/activate
cmake -B build
cmake --build build -j$(nproc)
cmake --install build
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

> **Update the robot IP addresses** in the demo source files to match your setup. The defaults are `192.168.1.2` (right arm) and `192.168.1.3` (left arm).

**C++ demos** are in `demos/cpp/`:

- `event_driven_teleop` — Event-driven dual-arm teleop using callback handlers. Press **A** to engage/disengage, **B** to exit. Triggers control grippers.
- `manual_polling_teleop` — Manual frame polling with inline edge detection. Same controls, different implementation pattern.

```bash
./build/event_driven_teleop
./build/manual_polling_teleop
```

**Python demos** are in `demos/python/`:

- `event_driven_teleop.py` — Python version of the event-driven demo.
- `manual_polling_teleop.py` — Python version of the manual polling demo.

```bash
source .venv/bin/activate
python demos/python/event_driven_teleop.py
python demos/python/manual_polling_teleop.py
```
