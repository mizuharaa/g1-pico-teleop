# One-time setup — status & remaining steps

System chosen: **HoloMotion v1.4.0** (Horizon Robotics) — full-body imitation
teleop, everything runs on the robot's Jetson in Docker. Research + decision
rationale: `~/.claude/plans/i-have-the-unitree-fuzzy-wombat.md`.

## ✅ Done (laptop, automated 2026-07-27)

- `HoloMotion/` repo cloned (v1.4.0, master).
- `artifacts/XRoboToolkit-PICO-1.1.1.apk` — headset app, ready to sideload.
- `artifacts/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` — downloaded.
- `artifacts/holomotion_v1.4.0_orin_jp5.1_arm64.tar` — the robot Docker image
  (9.1 GB, downloaded + tar-validated ✓, ready for offline install on the Jetson).
- `tools/platform-tools/` — adb (for the APK install).
- Conda env `holomotion_teleop` (py3.12) + XRoboToolkit SDK build — for the
  no-robot rehearsal + MuJoCo viewer. (If the build failed, rerun:
  `cd HoloMotion/deployment/holomotion_teleop && INSTALL_APT_DEPS=0 bash setup_holomotion_teleop_x86_ubuntu2204.sh`)

- PC Service deb installed on the laptop ✓ (roboticsservice 1.0.0, 2026-07-27).
- XRoboToolkit APK sideloaded on the PICO ✓ (`com.xrobotoolkit.client`).

## 🔲 Remaining — needs you / the hardware present

1. **Rehearsal (no robot) — ONLY THE TRACKERS ARE MISSING** (2026-08-07).
   Everything else is proven working — including the retargeter, which now
   runs ON THIS LAPTOP's CPU (2026-08-07: R_y180 parity fix + CPU fallback
   patched into `HoloMotion/holoretarget/`, verified end-to-end at a steady
   50 Hz, 5 ms/tick, via `--fake-pico-stream`; details in
   `bench/RESEARCH.md`, backups in `*.orig`).
   Bring the 3 trackers, wear them, then:
   ```bash
   # laptop internet: plug the phone in by USB and enable tethering FIRST
   #   (frees the Wi-Fi adapter; iPhone appears as e.g. enxea78656ce38f)
   nmcli device disconnect wlxd037457570db && nmcli con up g1-teleop-ap   # no sudo needed
   ~/full-body-teleoperation/scripts/start_rehearsal.sh            # T1: node + PC Service
   ~/full-body-teleoperation/scripts/start_viewer.sh local         # T2: MuJoCo G1
   ```
   In the headset: join Wi-Fi `g1-teleop` / `teleop12345` → open **XRoboToolkit**
   (Library → Unknown Sources) → click **Enter** next to `PC Service`, type
   **`10.42.0.1`** → **Reconnect** → Status goes green **WORKING**.
   Then tick Head + Controller + Send, and set **Mode → Full-body**.

   Verified on 2026-08-07: Status WORKING, PC Service 10.42.0.1, FPS 93.9,
   ping 18–43 ms, node reached "device found / TestDevice".
   **Blocker that stopped us: `Mode → Full-body` refuses to start without all 3
   trackers present ("3 trackers needed"), and the pucks were not on hand
   (`Num: 0`).** Body data never started, so the node stayed in its
   `waiting for body data...` loop and never bound ZMQ 6001.

   GOTCHAS:
   - The headset sleeps when off your head (`mWakefulness=Asleep`,
     `tracking_6dof_stopped`) — no tracking, no body data. Wear it.
   - Do NOT touch Wi-Fi in GNOME Settings → Network while the hotspot runs;
     `gnome-control-center` dropped the AP three times on 2026-08-06.
   - The app DOES have manual IP entry (the `Enter` button). It does not rely
     on broadcast discovery alone.
   Pass = the MuJoCo G1 mirrors your whole body (arms, waist, squat) smoothly.
   Do NOT go to the robot until this passes.
   Reminder: re-wearing the headset invalidates the calibration — redo it.
2. **Robot** (powered + Ethernet) — ONE COMMAND (2026-08-07):
   ```bash
   ~/full-body-teleoperation/scripts/deploy_to_robot.sh
   ```
   Does everything: image + installer + patched holoretarget (reference
   guard, R_y180 parity, IK tuning) + watchdog + e-stop, then the gate
   check. Manual equivalent kept for reference:
   ```bash
   cd ~/full-body-teleoperation
   scp artifacts/holomotion_v1.4.0_orin_jp5.1_arm64.tar scripts/robot_install.sh unitree@192.168.123.164:~/
   scp -r HoloMotion/holoretarget unitree@192.168.123.164:~/holoretarget_patched   # guard + parity + IK tuning
   scp scripts/joint_watchdog.py scripts/g1_estop.py unitree@192.168.123.164:~/    # safety tooling
   ssh unitree@192.168.123.164   # then: bash ~/robot_install.sh
   ```
   Gate: must print "HoloMotion Docker check PASSED. No robot action was sent."
   Also join the Jetson to the teleop router Wi-Fi (RUNBOOK §1) — the same Wi-Fi
   the PICO is on; note the Jetson's Wi-Fi IP, that's what the app points at.
3. **First live session**: RUNBOOK.md — gantry, hands removed, spotter on the
   remote, staged tests.

## Open questions to settle empirically

- Consumer PICO 4 Ultra body-streaming: expected to work (only camera capture is
  Enterprise-gated) — confirmed at rehearsal step 3.
- Teleoperated stepping/walking quality: architecturally supported, not
  officially claimed — evaluate on the gantry before free-standing walking.
- Jetson disk space for `docker load` (~25 GB needed): check `df -h` on the Orin.
