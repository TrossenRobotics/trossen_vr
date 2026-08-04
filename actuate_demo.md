# VR Teleop Demo — Actuate Conference Guide

Quick reference for running the VR teleop demo at the Actuate conference.

---

## 0. Get on the Right Branch & Build

Our actuate demo changes live on `dev/actuate_demo_main`.

```bash
git checkout dev/actuate_demo_main
cmake -B build -DBUILD_DEMOS=ON
cmake --build build -j$(nproc)
```

---

## 1. Check the Arm IP Addresses

IP address of each robot arm are hardcoded need to edit if needed.

- **Python demo:** open `demos/python/manual_polling_teleop.py`:
  ```python
  RIGHT_ARM_IP = "192.168.1.4"
  LEFT_ARM_IP = "192.168.1.5"
  ```
- **C++ demo:** open `demos/cpp/manual_polling_teleop.cpp` (same format).

If you change the IPs, save the file, then:

- **Python:** nothing else to do — just run it (step 5).
- **C++:** you must rebuild before the change takes effect:
  ```bash
  cmake --build build -j$(nproc)
  ```

### Smoothness tuning (`cmd_goal_time`)

This value controls how smooth vs. laggy the arm motion looks, and the right setting depends on the Wi-Fi latency on-site. `0.20` (seconds) is a good starting value.

- **Python:** `CMD_GOAL_TIME` in `demos/python/manual_polling_teleop.py`.
- **C++:** `cmd_goal_time` in `demos/cpp/manual_polling_teleop.cpp` — remember to rebuild (same command as above) if you change it.

If the arm looks jittery/jerky, try raising this number a bit. If it feels laggy/delayed, try lowering it slightly.

---

## 2. Connect the Headset to the Network

Pick **one** of the two options below.

### Option A (Preferred) — Dedicated Conference Router ("Rivet and Glide")

We're bringing our own Wi-Fi router to the conference which will be preferred over second option.

**On the System76 computer:**

1. Open **Settings → Wi-Fi**.
2. Click the gear icon next to **"Rivet and Glide"**.
3. Go to the **IPv4** tab and select **Manual**.
4. Fill in:
   - **Address:** `192.168.5.42`
   - **Netmask:** `255.255.255.0`
   - **Gateway:** `192.168.5.1`
   - **DNS:** `1.1.1.1`
5. Click **Apply**, then toggle the Wi-Fi/Ethernet off and back on.

**On the VR headset:**

1. Put on the headset and go to **Settings → Wi-Fi**.
2. Press connect button for **"Rivet and Glide"**.
3. Select **Advanced** option.
4. Under this section, change it from **DHCP** to **Static**.
5. Enter:
   - **IP Address:** `192.168.5.43`
   - **Gateway:** `192.168.5.1`
   - **Network Prefix Length:** `24` (same thing as `255.255.255.0`)
   - **DNS 1:** `1.1.1.1`
6. Confirm and enter password: Trossen2026.
7. In the Trossen VR Teleop app, enter this IP address in the **Robot PC IP Address** field (this is the computer's address not vr headset address):
   ```
   192.168.5.42
   ```
8. **Run the demo from the System76 computer** and *then* press **Connect** in the headset app.

### Option B (Fallback) — System76 Computer's Wi-Fi Hotspot

Use this only if the dedicated router isn't available.

1. Turn on the Wi-Fi Hotspot on the System76 computer (Settings > Wi-Fi > three dots near the top, close to the Wi-Fi on/off toggle).
2. On the headset: go to **Settings > Wi-Fi** and connect to that hotspot network.
3. Open the Trossen VR Teleop app and enter this IP address in the **Robot PC IP Address** field:
   ```
   10.42.0.1
   ```
4. **Run the demo from the System76 computer** and *then* press **Connect** in the headset app.

### Reduce Wi-Fi latency/jitter (do this on the System76 computer)

Wi-Fi power-saving can introduce lag/jitter in the arm movement. Turn it off with:

```bash
sudo iw dev wlp130s0f0 set power_save off
```

This resets after the computer restarts, so re-run it any time the computer has been rebooted.

---

## 3. Set the Headset's Display & Sleep Timers

Might have to do this after power cycle of the vr headset.

On the headset, go to **Settings > General** and set both of these to their maximum (4 hours):

- **Display Auto-Off** → 4 hours
- **Auto Sleep** → 4 hours

This keeps the headset from dimming or sleeping on its own in between demo runs.

---

## 4. Headset Battery & Proximity Sensor

The headset has a yellow sticker already placed inside it over the proximity sensor. This tricks the headset into staying awake even when it's taken off.

- **Do not press the power button** — this can still put the headset to sleep even with the sticker in place.
- **This trick stops working below ~20% battery.** Keep the headset charging any time it's not actively in someone's hands or maybe all the time if power is nearby, so it doesn't drop below that level mid-demo.

---

## 5. Start the Demo

**Run this on the System76 computer first — then go press Connect on the headset app**.

Only the **manual polling** demo is used for the conference demos (not the event-driven one).

- **Python:**
  ```bash
  uv run demos/python/manual_polling_teleop.py
  ```
- **C++:**
  ```bash
  ./build/manual_polling_teleop
  ```

Either one works the same way.

---

## 6. Controls (for the person doing the teleop)

- **Hand/Grip Trigger:** hold to control the arm, release to pause.
- **Index Trigger:** open/close the gripper.
- **A Button (right controller):** recover an arm that's faulted (e.g. hit a joint limit) — it moves back to home, then resumes teleop.
- **B Button (right controller):** exit the program — arms will move to their idle position automatically.
- **Meta Button (right controller):** press to snap/re-align the UI window to right in front of your current view — useful if the panel ends up out of sight or at an awkward angle.

---

## Quick Troubleshooting

- **App won't connect:** confirm the demo program is already running on the System76 computer *before* pressing Connect on the headset — pressing Connect first won't work. Also double check headset and computer are on the same network, and that the IP typed into the app is correct.
- **Headset sleeps when removed:** check the yellow sticker over the proximity sensor hasn't shifted or fallen off or check step 3.
