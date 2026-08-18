# Unitree G1 + PICO 4 Ultra — FULL-BODY Teleoperation Runbook (HoloMotion v1.4.0)

Read offline: `less ~/full-body-teleoperation/RUNBOOK.md`.
Simple version: `INSTRUCTIONS.md`. One-time installs: `SETUP_STATUS.md`.

The robot IMITATES YOUR WHOLE BODY (arms, waist, squat, posture) from the PICO
headset + 2 wrist-held controllers + 2 ankle trackers. Everything runs ON THE
ROBOT's Jetson (Docker); the laptop is only an optional viewer.

====================================================================
## 0. HOW THIS DIFFERS FROM THE QUEST SETUP (~/meta-quest-teleoperate)
====================================================================
- No camera in the headset: you operate by DIRECT LINE OF SIGHT. Do not wear
  the headset fully blind — pivot it up slightly or use passthrough view.
- No laptop in the control loop; the PICO talks straight to the robot over Wi-Fi.
- REMOVE THE INSPIRE HANDS before running the policy (maker's safety guidance:
  the policy was trained/tested without distal hand mass). Hands come back for
  the Quest/xr_teleoperate manipulation setup.
- The robot BALANCES ITSELF and can STEP. This is a legged-control session, not
  an arms-only session: gantry first, spotter mandatory.

====================================================================
## 1. THE TWO NETWORKS
====================================================================
- Laptop <-> ROBOT  = Ethernet cable (192.168.123.x, laptop = .2, robot = .164).
- PICO <-> ROBOT    = the teleop ROUTER Wi-Fi (5 GHz). The ROBOT's Jetson must be
  on that Wi-Fi too (NEW — the Quest setup never needed this).

Join the robot to the router Wi-Fi (once per router):
```bash
ssh unitree@192.168.123.164        # ROS prompt: 1
nmcli device wifi connect "<SSID>" password "<PSK>"
hostname -I                        # note the robot's Wi-Fi IP, e.g. 192.168.1.x
```
The PICO app will point at that Wi-Fi IP.

====================================================================
## 2. SESSION START (every time)
====================================================================
1. HANDS OFF: unbolt/remove both Inspire hands.
2. Hang the robot on the GANTRY, feet just off the ground. Clear 3x3 m area.
3. Power on; plug the Ethernet cable; `ping -c2 192.168.123.164`.
4. Wait for zero-torque/idle, then on the Unitree remote press **L2+R2**
   (debug mode — joints go to damping; Unitree's own control services stand down).
5. Designate the SPOTTER: holds the Unitree remote the whole session.
   Emergency actions: **Select** (HoloMotion e-stop) / remote damp / power switch.

====================================================================
== 3. START THE CONTROLLER (on the robot)
====================================================================
```bash
ssh unitree@192.168.123.164
docker run --rm -it \
  --runtime nvidia --gpus all --privileged --network host \
  --name holomotion_g1 --entrypoint bash \
  horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64
# inside the container:
holomotion check      # MUST end with: "HoloMotion Docker check PASSED. No robot action was sent."
holomotion teleop     # starts policy + the on-robot XRoboToolkit service
```
Logs: `tail -f /tmp/holomotion_pico_service.log` (want: "Pico service ready").
If the robot's DDS interface isn't eth0, edit
`deployment/unitree_g1_ros2_29dof/launch_profiles/orin_docker.yaml`
(`network_interface:`) — never the ROS launch files.

====================================================================
## 4. CONNECT THE PICO
====================================================================
1. Trackers on both ankles (light up/outward, not covered), controllers in hand.
2. Headset on the router Wi-Fi. Open **XRoboToolkit** app.
3. **PC Service = robot's Wi-Fi IP** (Section 1) → status **WORKING**.
4. Enable **Head / Controller / Full body / Send**.
5. Stand in a stable neutral pose until body tracking shows. If you re-wear the
   headset later → REDO the PICO body calibration (bad calibration = robot
   contorts, see Section 7).

====================================================================
## 5. OPERATE (controller buttons — HoloMotion FSM)
====================================================================
ALL buttons are on the UNITREE REMOTE (read from lowstate.wireless_remote),
NOT the Pico controllers (verified in main_node.cpp, 2026-08-10):

| Button | Action |
|---|---|
| **START** | ZERO_TORQUE → MOVE_TO_DEFAULT: robot stiffens, rises to default pose |
| **A** | (only after START) enter POLICY: balance on, velocity mode |
| **B** | START whole-body motion tracking (imitates you) |
| **Y** | Back to velocity mode (joystick walking, no imitation) |
| **Select** | EMERGENCY STOP |
| **L1** | Back to ZERO_TORQUE |

Sequence for a run:
1. Press **A** → robot to default pose → lower the gantry until feet take weight,
   keep slack in the straps.
2. Stand NEUTRAL AND STILL, then press **B**. If logs say "VR queue is not
   ready", wait for streaming and press **B** again.
3. Staged tests, in order, one per run early on:
   shift weight → slow arm moves → shallow squat → step in place.
   Escalate only after the previous stage is stable.
4. Stop: **Y** → **Select** → Ctrl+C in the container.

!!! AUTOMATIC FAILSAFE (2026-08-10 rework): on stream loss (Wi-Fi drop,
headset sleep/off-head) OR sustained impossible input (> 2 s of thrown
controller, tracker glitch, solver blow-up) the robot AUTOMATICALLY returns
to velocity mode = default standing pose. Transient glitches (< 2 s) are
bridged by holding the last good reference. Recovery is manual by design:
fix the input (re-tick Full body + Send in the app — Mode resets to None on
every reconnect), stand NEUTRAL, press B.
The spotter with the remote is still MANDATORY: the failsafe depends on the
software stack being alive, damp = collapse, and there is NO hardware e-stop.

Optional laptop viewer (watch the reference the robot follows):
```bash
~/full-body-teleoperation/scripts/start_viewer.sh          # via Ethernet
```

====================================================================
## 5b. THERMAL DISCIPLINE (learned 2026-08-10: waist_pitch hit 130 C)
====================================================================
- The WAIST PITCH motor cooks when the reference holds a constant torso
  lean: check the WAIST PUCK SITS FLAT ON THE SACRUM before every run —
  a tilted puck = permanent lean = static ~30 Nm on waist_pitch.
- Watchdog WARNs at 70 C, ALARMs + auto-damps at 85 C (re-arms every 10 s).
  At any waist WARN: finish the move, go to velocity mode (Y), rest 5 min.
- Session pacing: ~15 min tracking, then a standing/velocity break.
- Motor windings age fast above ~100 C — 130 C events cost robot lifetime.

====================================================================
## 6. SHUT DOWN
====================================================================
1. **Y** (velocity mode, robot stands) → hoist gantry snug → **Select**.
2. Ctrl+C in the container; `exit`.
3. Power off / normal Unitree shutdown. Re-fit hands only for manipulation days.

====================================================================
## 7. TROUBLESHOOTING
====================================================================
- **App won't reach robot / not WORKING**: robot and PICO on same Wi-Fi? `ping`
  the robot Wi-Fi IP from the laptop on that Wi-Fi. Firewall on Orin? (none by
  default). PC Service runs inside the container — `holomotion teleop` must be up.
- **"VR queue is not ready"**: streaming not started (Send enabled?), or app not
  in WORKING state. Fix, then press **B** again.
- **Robot pose is wrong/contorted after B** (elbow pinned, waist twisted):
  known failure mode of bad calibration (upstream issue #21). Recalibrate PICO
  full-body tracking; do NOT re-wear the headset after calibrating; be in a
  neutral stance BEFORE pressing B; switch directly velocity→tracking.
- **`holomotion check` fails on nvidia runtime**: rerun
  `~/robot_install.sh` (configures /etc/docker/daemon.json) — see scripts/.
- **No DDS / robot ignores commands**: debug mode not entered (L2+R2), or wrong
  `network_interface` in orin_docker.yaml.
- **Trackers drift/drop**: light side up, ankles visible to headset cameras,
  tight pants, good lighting. Re-pair in Settings → Devices → Motion Tracker.
- **Jerky/latent**: 5 GHz Wi-Fi, robot close to router, no wall between.

====================================================================
## 8. FALLBACKS (if HoloMotion disappoints)
====================================================================
1. **TWIST2** (github.com/amazon-far/TWIST2): laptop-in-loop, ships pretrained
   ONNX (CPU-only untested; use `--device cpu` + plain `onnxruntime`), same
   PICO+tracker kit. Line-of-sight only. Stale repo — expect rough edges.
2. **xr_teleoperate v1.5** (Unitree): PICO 4 Ultra supported, Inspire FTP hands
   supported, camera-in-headset works — but legs = joystick walking only
   (`--motion` mode). Best manipulation track; reuse ~/meta-quest-teleoperate
   knowledge.

====================================================================
## 9. KEY VALUES
====================================================================
Robot PC2 (Ethernet) 192.168.123.164 | laptop wired .2 | robot Wi-Fi IP: per router
Ports: 6002 robot->viewer telemetry | Image: horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64
Buttons: A=default pose, B=track me, Y=velocity mode, Select=E-STOP
Remote: L2+R2 = debug mode | Damp + power switch = last resort (no hardware e-stop!)
Conda env (laptop): holomotion_teleop | APK: XRoboToolkit-PICO-1.1.1
