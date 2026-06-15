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
uv sync

# Configure with Python bindings enabled
source .venv/bin/activate
cmake -B build -DBUILD_PYTHON=ON

# Build and install Python module
cmake --build build -j$(nproc)
sudo cmake --install build
```

## VR Headset App

### Installing the APK

The Trossen VR Teleop app (`assets/VR_Teleop.apk`) can be sideloaded onto a Meta Quest headset. First, enable Developer Mode on the headset (Settings > System > Developer), then use one of the methods below.

**Using Meta Quest Developer Hub — Windows only:**

1. Download and install [Meta Quest Developer Hub](https://developers.meta.com/horizon/downloads/package/oculus-developer-hub-win/)
2. Connect the headset to your PC via USB
3. In MQDH, go to **Device Manager** > **Apps** > **Install APK** and select `assets/VR_Teleop.apk`

**Using ADB — Linux / Ubuntu:**

1. Install ADB:
   ```bash
   sudo apt install adb
   ```

2. Fix permissions (required on first use):
   ```bash
   # Create udev rules for Meta Quest
   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/51-android.rules
   sudo chmod a+r /etc/udev/rules.d/51-android.rules

   # Reload udev rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger

   # Restart ADB if running and reconnect headset
   adb kill-server
   adb start-server
   ```
   Disconnect and reconnect the headset USB cable after running these commands.

3. Connect the headset to your PC via USB. Put the headset on — a prompt will appear inside asking you to Allow USB Debugging. Select Always allow from this computer and confirm.

4. Verify the headset is detected:
   ```bash
   adb devices
   ```
   You should see a device listed with status `device`. If it shows `unauthorized`, re-check the Allow USB Debugging prompt inside the headset.

5. Install the APK:
   ```bash
   adb install assets/VR_Teleop.apk
   ```
   A `Success` message confirms the installation completed.

> [!NOTE]
> The app will appear in the headset's App Library. If it is not visible, switch the library filter to Unknown Sources.

### App UI

![VR Teleop App UI](assets/VR_Teleop_ui.jpg)

| Element | Description |
|---------|-------------|
| **Robot PC Address field** | IP address of the PC running this `trossen_vr` library |
| **Connect** | Connect to the robot PC and start streaming arm data |
| **Disconnect** | Stop streaming (robot arm holds its last position) |
| **Status** | Live connection state: `Disconnected` → `Connecting` → `Connected` / `Degraded` |
| **Frequency** | Data receive rate in Hz (shown once connected) |
| **Passthrough** | Toggle camera passthrough view |
| **Quit** | Exit the application |

Controller shortcuts:

- **Left Menu Button**: Show or hide the UI panel
- **Right Meta Button**: Snap the UI panel to in front of your current view
- **Grip / Hand Trigger (deadman switch)**: Hold to enable tracking and engage the arm. Release to pause.

### Configuring the IP Address

Enter the IP address of the PC running the trossen_vr application or demo in the **Robot PC IP Address** field, then press **Connect**.
The headset and PC must be on the same network.

### Ways of Operating

Once connected, there are two ways to use the app:

#### Passthrough Mode

Press the Passthrough button in the UI to enable the headset's cameras so you can see the real world around you. The UI panel will disappear when passthrough is active — press the Left Menu Button on the left controller to bring it back.

#### Direct View (Headset Removed)

Remove the headset and place it somewhere with a clear view of the controllers — overhead is recommended. This lets you observe the robot directly without a screen.

> [!WARNING]
> **Proximity sensor limitation:** Meta does not currently provide a built-in option to disable the proximity sensor from within the app — the headset will go to sleep immediately when removed. We will update the app once Meta adds support for this. In the meantime, two workarounds are available:
>
> - **Meta Quest Developer Hub — Windows only (up to 8 hours)**: Connect the headset to your PC via USB, open [Meta Quest Developer Hub](https://developers.meta.com/horizon/downloads/package/oculus-developer-hub-win/), and disable the proximity sensor under Device Manager > Device actions. This keeps the display on for up to 8 hours.

---

## Demos

> [!IMPORTANT]
> **Before running any demo**, start the Trossen VR Teleop app on your Meta Quest headset, enter the IP address of the PC, and press Connect Button. The headset and PC must be on the same network.

> [!TIP]
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
uv run demos/python/event_driven_teleop.py
uv run demos/python/manual_polling_teleop.py
```
