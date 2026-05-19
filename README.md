# Trossen VR Teleoperation

C++ and Python SDK for controlling Trossen robot arms using Meta Quest VR controllers over UDP.

## Requirements

- [Eigen3](https://eigen.tuxfamily.org/) >= 3.3
- [nlohmann/json](https://github.com/nlohmann/json) >= 3.2
- [libtrossen_arm](https://docs.trossenrobotics.com/trossen_arm/)
- [pybind11](https://github.com/pybind/pybind11) >= 2.11 (for Python bindings)
- Python >= 3.11, [uv](https://docs.astral.sh/uv/)

```bash
sudo apt install -y cmake build-essential libeigen3-dev nlohmann-json3-dev
```

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

## Demos

> **Before running any demo**, start the Trossen VR Teleop app on your Meta Quest headset. In the app, enter the IP address of the PC running the demo and press **Connect**. The headset and PC must be on the same network.

> **Update the robot IP addresses** in the demo source files to match your setup. The defaults are `192.168.1.4` (right arm) and `192.168.1.2` (left arm).

**C++ demos** are in `demos/cpp/`:

- `dual_arm_teleop` — event-driven dual-arm teleop using `Teleop` callbacks. Press **A** to engage/disengage teleop, **B** to exit. Triggers control the grippers.
- `manual_polling_teleop` — manual frame polling with inline edge detection. Same controls as above but uses explicit polling instead of callbacks.

```bash
./build/dual_arm_teleop
./build/manual_polling_teleop
```

**Python demos** are in `demos/python/`:

- `dual_arm_teleop.py` — Python version of the event-driven demo.
- `manual_polling_teleop.py` — Python version of the manual polling demo.

```bash
source .venv/bin/activate
python demos/python/dual_arm_teleop.py
python demos/python/manual_polling_teleop.py
```