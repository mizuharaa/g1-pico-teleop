# 🤖 Full-Body Teleoperation — Simple Instructions

Make the **Unitree G1** copy your **whole body** (arms, waist, squats, steps)
using the **PICO 4 Ultra** + **2 ankle trackers**.

> Details & troubleshooting: `RUNBOOK.md`. First-time installs: `SETUP_STATUS.md`.

---

## 🔌 Before you start

| | |
|---|---|
| ✋ | **Remove both robot hands** (safety rule of the software maker). |
| 🏗️ | Hang the robot on the **gantry**, feet just off the ground, 3×3 m clear. |
| 🔋 | Robot on + **Ethernet cable** in. Remote: press **L2+R2** (debug mode). |
| 📶 | Router on (5 GHz). The **robot's Wi-Fi** and the **PICO** join it. |
| 🦺 | A **second person holds the Unitree remote** the entire time. |

> ⚠️ There is **no camera in the headset** in this setup — keep the robot in
> direct sight. And there is **no auto-stop** if Wi-Fi drops: the spotter is
> the emergency stop.

---

## ▶️ Step 1 — Start the robot controller

```bash
ssh unitree@192.168.123.164        # ROS prompt: 1
docker run --rm -it --runtime nvidia --gpus all --privileged --network host \
  --name holomotion_g1 --entrypoint bash \
  horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64
holomotion check                   # must say: check PASSED. No robot action was sent.
holomotion teleop
```

## 🥽 Step 2 — Gear up

1. Strap trackers to **both ankles** (light facing out, not covered by pants).
2. Hold the controllers. Put on the headset (tilted so you can still see the robot).
3. Open **XRoboToolkit** app → **PC Service = robot's Wi-Fi IP** → wait for **WORKING**.
4. Turn on **Head + Controller + Full body + Send**.
5. Stand still and neutral until your body shows up.

## 🕹️ Step 3 — Drive

| Button | Does |
|---|---|
| **A** | Robot goes to its ready pose (still hanging) |
| **B** | Robot starts **copying you** |
| **Y** | Stop copying → joystick walking mode |
| **Select** | 🛑 EMERGENCY STOP |

1. Press **A** → lower the gantry until the feet carry weight (keep straps slack).
2. Stand **neutral and still** → press **B** → move slowly. First sessions:
   weight shift → arms → shallow squat → stepping in place. One new thing at a time.
3. Done: **Y** → hoist snug → **Select** → Ctrl+C.

---

## 🆘 Quick fixes

| Problem | Do |
|---|---|
| App stuck, not WORKING | Robot Wi-Fi and headset on the same router? `holomotion teleop` running? |
| "VR queue is not ready" | Enable **Send** in the app, wait, press **B** again |
| Robot pose crooked after B | Redo PICO body calibration; don't re-wear headset after; stand neutral before B |
| Trackers lost | Light side out, ankles visible, tight pants, re-pair in Settings → Devices |
