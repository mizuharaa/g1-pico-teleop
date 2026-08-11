# PICO 4 Ultra setup (one-time)

Goal: install the XRoboToolkit app and pair the 2 ankle motion trackers.

## 1. Enable developer mode + USB debugging (consumer PICO 4 Ultra)

1. In the headset: **Settings → General → About** → click **Software Version** 7 times
   → "Developer" appears under Settings → General.
2. **Settings → General → Developer** → enable **USB Debugging**.

## 2. Install the XRoboToolkit app

Plug the headset into the laptop with USB-C, put the headset ON (accept the
"Allow USB debugging" prompt inside the headset), then on the laptop:

```bash
cd ~/full-body-teleoperation
./tools/platform-tools/adb devices          # must list the device (not "unauthorized")
./tools/platform-tools/adb install -g artifacts/XRoboToolkit-PICO-1.1.1.apk
```

The app appears in the headset library under **Unknown sources** as "XRoboToolkit".

## 3. Pair the motion trackers (2 of your 3)

1. Headset: **Settings → Devices → Motion Tracker** (PICO's tracker pairing UI),
   power on two trackers, pair them.
2. Strap them to both **ankles**, light indicator facing up/outward, visible —
   scrunch baggy trouser legs above them (tight-fitting pants recommended).
3. Run PICO's **full-body tracking calibration** when prompted (stand straight,
   look ahead). Redo calibration if you remove/re-wear the headset.

## 4. Connect to the robot (each session)

1. Headset Wi-Fi → the teleop router (5 GHz) — same network as the robot's Orin.
2. Open the **XRoboToolkit** app.
3. Set **PC Service** to the **robot's Wi-Fi IP** (see RUNBOOK section 2).
4. Status must show **WORKING**.
5. Enable **Head**, **Controller**, **Full body**, and **Send**.
6. Stand still in a neutral pose until body tracking is visible.

Controllers are held (or strapped to the wrists); trackers on ankles.

## Notes

- Consumer (non-Enterprise) PICO 4 Ultra: body/tracker streaming works; only
  camera passthrough capture is Enterprise-gated (we don't use it).
- If the app can't connect: check both devices are on the same subnet, and that
  no firewall runs on the Orin.
