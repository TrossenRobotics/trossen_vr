# Trossen VR Teleoperation

C++ and Python SDK for controlling Trossen robot arms using Meta Quest VR controllers over UDP.

## Requirements

- CMake >= 3.15, C++17 compiler
- [Eigen3](https://eigen.tuxfamily.org/) >= 3.3
- [nlohmann/json](https://github.com/nlohmann/json) >= 3.2
- [libtrossen_arm](https://docs.trossenrobotics.com/trossen_arm/)
- [pybind11](https://github.com/pybind/pybind11) (for Python bindings)
- Python >= 3.11, [uv](https://docs.astral.sh/uv/)

```bash
sudo apt install -y cmake build-essential libeigen3-dev nlohmann-json3-dev pybind11-dev
```

## Build

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## Python Setup

```bash
uv venv --python 3.11
uv sync
```

## Demos

**C++ demos** are in `demos/cpp/`:

- `dual_arm_teleop` — event-driven dual-arm teleop with `Teleop` callbacks
- `manual_polling_teleop` — manual frame polling with inline edge detection
- `receiver` — prints incoming VR frames (test utility)

```bash
./build/dual_arm_teleop
./build/receiver
```

**Python demos** are in `demos/python/`:

```bash
source .venv/bin/activate
PYTHONPATH=build python demos/python/dual_arm_teleop.py
```

## Architecture

```
include/trossen_vr/
  vr_types.hpp          — Vec6, ControllerPose, VRFrame, transforms, JSON parser
  network_manager.hpp   — UDPReceiver (threaded, newest-frame-wins)
  teleop.hpp            — Teleop dispatcher, TeleopConfig

src/
  network_manager.cpp   — UDPReceiver implementation
  teleop.cpp            — Teleop dispatch (edge detection, handler firing)
  bindings.cpp          — pybind11 module (trossen_vr)
```

The Unity app (Meta Quest 3) sends JSON over UDP port 9000 containing controller poses and button states. `UDPReceiver` runs a background thread that drains the socket and keeps only the latest frame. `Teleop` dispatches button events (rising-edge for bools, every-frame for analogs) and pose updates via registered callbacks.