# Trossen VR Teleoperation SDK

This repository provides a VR teleoperation framework for controlling Trossen robotic arms using VR controllers. It includes:

- **C++ SDK** for manual and event-driven teleoperation.
- **Python bindings** for scripting VR teleoperation in Python.
- Integration with Trossen Arm drivers (`libtrossen_arm`) for robot control.

---

## Table of Contents

1. [Build Instructions](#build-instructions)  
   - [Dependencies](#dependencies)  
   - [Build C++ SDK](#build-c-sdk)  
   - [Build Python Bindings](#build-python-bindings)  
2. [Running C++ Scripts](#running-c-scripts)  
3. [Running Python Scripts](#running-python-scripts)  
4. [Example Scripts](#example-scripts)  
5. [Notes](#notes)  

---

## Build Instructions

### Dependencies

Ensure the following libraries are installed:

- **C++**:  
  - CMake >= 3.16  
  - Eigen3  
  - pthread  
  - libtrossen_arm (Trossen arm driver library)

- **Python** (optional for Python bindings):  
  - Python 3.10+  
  - pybind11  
  - NumPy  

---

### Build C++ SDK

1. Clone the repository:

```bash
git clone https://github.com/TrossenRobotics/trossen_vr.git
cd trossen_vr
```

2. Create a build directory and navigate into it:

```bash
mkdir build
cd build
```

3. Generate build files with CMake

``bash
cmake ..
```

4. Build the SDK

```bash
make install
```

This will generate:
- Executable files for C++ teleoperation scripts
- Shared library for Python bindings (pytrossen_vr.cpython-<version>-x86_64-linux-gnu.so)

### Build Python Bindings

The Python bindings are automatically built with the C++ build if pybind11 is installed.
After building, the .so file is located in the build/ folder:

