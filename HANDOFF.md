# HANDOFF — 2026-08-11 evening: LIVE headset->sim teleop achieved

Operator drove the sim G1 with their body (squats, kicks tracked). Final
sim architecture — zero ceremony, fully self-healing:
- `sim_with_pico.sh` = keeper loop: relaunches the sim on crash/freeze
  (heartbeat file /tmp/holosim_heartbeat, 20 s), sleep/wake detected via
  wall-clock jump -> in-place reset; ESC/window-close = deliberate quit.
- Hands-free: stream live -> tracking ON (~1 s); stream dead -> velocity
  stand; robot falls -> teleport reset + 2 s cooldown. NO keyboard, NO
  pose gate (gates fought the operator; resets are free in sim).
- Smoothness knobs (sim-validated, DEFAULT OFF on robot):
  HOLOMOTION_TARGET_SLEW_RAD_S=15 (8 FELL in suite — never tighten
  without re-running it) and HOLOTELEOP_REF_SMOOTH=0.4 (EMA at the
  retarget node vs 91->50 Hz aliasing, ~13 ms latency).
- Behavior verdicts: micro-stepping reduced (jitter+slew fixes), one-leg
  static balance = partly policy limit, jumping = checkpoint CANNOT
  (grounded training data) -> cloud finetune with jump clips is the path.

# HANDOFF — 2026-08-11 late PM: sim-teleop live path + hard lessons

## 🔴 ROBOT BLOCKER FOUND: cpu backend vs pico_local (task #6)
The 08-10 anti-fall fix (`motion_observation_backend: "cpu"`) makes the
robot's policy node setup FAIL: `_init_gpu_motion_observation` raises
"reference_source=pico_local requires Warp CUDA Observation"
(policy_node_29dof.py:519). FSM + PC service still come up, so it LOOKS
alive — but tracking can never start. Fix options in task #6. Robot image
otherwise ready: all patches + TRT cache baked (container start -> policies
loaded in 50 s now, was 8-10 min).

## Sim teleop (headset -> laptop MuJoCo) WORKS — bench/simsmoke/sim_with_pico.sh
Verified end-to-end at 50 Hz: headset (g1-teleop AP, app -> 192.168.123.2)
-> robot NAT -> laptop PC service -> CPU retarget -> zmq 6001 -> interactive
sim. AUTO-TRACK: stable stream = tracking starts itself (headset off-head
kills the stream, so keyboard-triggered B was unusable solo).

## Ops lessons (paid in hours today)
- PC service ran as a ZOMBIE since 08-10 morning: listening on 63901 but
  dead inside after laptop sleep. Head-pose probe (in teleop node) now
  disambiguates: head_pose_ok = app connected, Mode not Full-body;
  head_pose_dead = no link/wedged. Supervisor recycles the service ONLY on
  positive dead evidence (3+ dead probes, zero ok) — absence-based logic
  recycled it mid-connect and reset the app's Mode in a loop.
- SDK latch bug: after sleep is_body_data_available() stays true with
  frozen timestamps — PicoReader now reports instead of spinning silently.
- Process pile-up: relaunches stacked supervisors/nodes; extra nodes hold
  the ONE SDK client slot -> zmq silent. Supervisor now flock-guarded +
  kills stray nodes each cycle. NEVER pkill by pattern that appears in
  your own command line (self-kill, exit 143/144 — burned us 4x).
- Laptop must NEVER join g1-teleop (no internet; roaming broke API access).
  Headset->laptop path is via robot NAT: app IP 192.168.123.2 for SIM,
  10.42.0.1 only for real robot runs. ONE SDK client per PC service.

# HANDOFF — 2026-08-11 PM: smoke suite green, patches DEPLOYED to robot

**Robot was found powered ON + reachable this session (someone powered it
up — CONFIRM the gear inspection actually happened before any standing).**

## ✅ MuJoCo smoke-test env BUILT and 6/6 gating scenarios PASS
`bench/simsmoke/` — runs the REAL deployed stack (actual PolicyRuntime +
obs pipeline + deployed ONNX checkpoints, CPU) against a faithful Python
port of main_node.cpp + MuJoCo `scene_29dof.xml` (missing waist/torso
meshes fetched from unitree_ros; sim2sim mesh blocker from TODO#4 CLEARED).
Run: `conda activate holomotion_teleop && python bench/simsmoke/run_smoke.py`
- PASS startup ladder -> velocity free-stand (h_min .756, tilt 1.9°)
- PASS B-entry soft-start + 2 squat-cycle tracking
- PASS Y-exit blend (0.024 rad max step; 0.525 rad snap with blend off —
  the suite CATCHES the regression, A/B via HOLOMOTION_MODE_BLEND_S=0)
- PASS stale-reference auto-fallback (blended, no snap, no fall)
- PASS Select e-stop sequence; PASS L1 patch (peak collapse speed 8.8
  vs 18.0 rad/s upstream free-fall)
- INFO impossible-pose (unguarded zmq path FALLS -> guard stays mandatory)
- INFO 2.65 s loop stall REPRODUCES the 08-10 fall in sim (root cause
  independently confirmed)
Sim-vs-real gaps to keep in mind: CPU ORT (no TRT), sim clock injected for
determinism, gantry = soft pelvis spring, contact params unvalidated.
FINDING for the twitch hunt: motion policy commands ankle target steps up
to ~0.5 rad/20 ms while balancing — raw output, no smoothing anywhere.

## ✅ 2026-08-11 patches are IN THE ROBOT IMAGE (deploy_cpp_patch.sh)
Rebuilt humanoid_control inside a build container (targeted colcon build,
NO clean; models verified intact), committed, gate check PASSED.
- mode-blend (policy_runtime.py) — sim-verified
- L1 -> damped e-stop; L1+L2 chord = old instant-limp for bench use
  (main_node.cpp, VERIFIED via strings on the rebuilt binary)
Still pending on robot: TRT engine cache re-bake (first controller start
recompiles ~8-10 min), spotter-guarded live tracking retest.

## ✅ New one-command bringup: `scripts/session_up.sh`
Idempotent check->fix->recheck phases: disk, wired link, ssh, docker
daemon+image, image-yaml sanity, robot AP (now sets autoconnect!), PC
service port, container, watchdog. `--check-only` / `--no-app` flags.
Discovered: PC service runs INSIDE the app container
(`pico_service_command` in orin_docker.yaml) — port 63901 only listens
once the container is up; session_up warns accordingly.

# HANDOFF — 2026-08-11 AM (robot off; code + FSM audit only)

## Select e-stop SOURCE-VERIFIED (was TODO #3) + remote button map in POLICY
From `src/src/main_node.cpp` (FSM node), buttons read via lowstate — the
Unitree remote IS live under custom control for these:
- **Select** = EMERGENCY_STOP: 2 s pure damping (kp=0, kd=10) → motors
  **DISABLED** (mode=0, zero everything) → `rclcpp::shutdown()`. Ends in
  full collapse + dead node; restart required. Use only while supported.
- **⚠️ L1 = INSTANT ZERO_TORQUE from ANY state** (kp=kd=0, free-fall —
  WORSE than damp). Never press L1 while the robot is standing. Trap!
- **Y** (in tracking/motion mode) = switch back to velocity mode — the
  velocity policy is a self-balancing stand. This is the graduated soft
  stop that already exists; it should be the FIRST response, not damp.
- Start only works in ZERO_TORQUE; A only in MOVE_TO_DEFAULT (with a
  0.4 rad lower-body deviation gate). There is NO path POLICY →
  MOVE_TO_DEFAULT.

## New LOCAL PATCH (2026-08-11, UNVERIFIED on hardware): mode-blend
`policy_runtime.py` — tracking→velocity re-entry (Y press OR stale-data
auto-fallback) used to snap the commanded target from tracked pose to
default pose in ONE 50 Hz step (instant gain swap + target jump = jerk;
likely a twitch source at every max_data_age bounce). Now blends last
commanded pose → new target, smoothstep over `HOLOMOTION_MODE_BLEND_S`
(default 1.5 s), mirroring the 08-10 B-press soft-start (applied after
actions_onnx is stored, so policy obs untouched). 15/15 unit tests pass
(3 new). TODO: rehearsal-verify in MuJoCo, then overlay to robot image
together with the orin_docker.yaml cpu-backend fix.

## Twitching — not yet root-caused (robot off). Ranked suspects
1. Mode flapping (max_data_age bounces) — each bounce = instant gain swap
   + target snap. 1.5 s age fix + new blend patch address this.
2. NO smoothing anywhere: raw policy action × scale + default goes
   straight to motor q at full policy kp. Obs/reference noise (2.4 GHz
   hiccups, PICO jitter) passes straight through. Do NOT blind-add an EMA
   filter — action history feeds policy obs; characterize first.
3. Loop-timing micro-stalls (same class as the fall): `policy_total_ms`
   timing already instrumented — pull spikes from logs of run 1/2.
Diagnosis without standing: reproduce in laptop MuJoCo rehearsal
(--fake-pico-stream or trackers), record target_dof_pos jitter; compare
run-1 footage timestamps vs logs; watchdog CSV tau oscillations.

---

# HANDOFF — 2026-08-10 end of day (see 2026-08-07 sections below)

**One-line status: first live session ran (velocity mode GOOD, run-1 footage
on tape); first tracking-mode entry FELL (root-caused + fixed, unverified);
robot powered off mid-setup for run 2 — INSPECT GEARS before next power-on.**

## 2026-08-10 session log

### 🔴 Before ANYTHING next session
- **Operator heard "motors/gears clashing" while handling the powered-off
  robot after the fall.** Hand-check every joint that took the impact (slow
  back-drive, compare left/right). A grinding/notchy joint = stop, open it up.
- Robot fell once from standing (damp-equivalent, ~3 s into tracking mode).
  Survived: rebooted fine afterward, then was intentionally powered off.

### The fall — root cause (proven from logs, fix in repo but UNVERIFIED)
1. B-press → motion mode → `holoretarget._gpu_targets` Warp kernel **JIT
   compiled 2.65 s on first use**, freezing the 50 Hz loop (7.7 Hz actual).
2. Even compiled, GPU staging ran ~46-52 ms/step (~20 Hz). Robot fell ~3 s in.
3. Cause: `motion_observation_backend: "auto"` picked the unrehearsed Warp GPU
   path on Orin. **Fixed to `"cpu"`** (rehearsal-validated, 50 Hz / 5 ms) in
   `launch_profiles/orin_docker.yaml`. NOT yet overlaid into the robot image —
   the scp/overlay chain was killed when the operator started rewiring.
   Overlay + gate-check + a spotter-guarded tracking test are still TODO.

### ✅ Wi-Fi blocker RESOLVED — robot-hosted AP works end-to-end
- `sudo nmcli device wifi hotspot ifname wlan0 ssid g1-teleop password
  teleop12345` on the Orin (NM profile `Hotspot-1`, autoconnect NOT yet set).
- Headset associated, app PC-service `10.42.0.1` port 63901 connected,
  policy logged "Live reference is ready". First topology to pass all day.
- Laptop AP `g1-teleop-ap` is confirmed dead-end (assoc now works but app
  path over NAT doesn't) — do not retry it; profile still exists, leave down.

### ⚠️ Traps discovered (cost hours)
- **`build_before_launch: true` in the repo's orin_docker.yaml wipes
  `install/` (incl. model weights!) on every container start** — models only
  exist in install/, so launch dies with "No config file found in
  velocity_tracking_model". Our yaml now says `false`; NEVER ship the yaml
  without it. This masqueraded as image corruption for ~45 min.
- Do NOT `docker commit` while the app runs; overlay via stopped
  `docker create` + `docker cp` + `commit` (robot_install.sh + fast path in
  deploy_to_robot.sh now both ship the yaml).
- `max_data_age` raised 0.6→1.5 s (2.4 GHz hiccups were bouncing tracking
  back to velocity mode seconds after B). In image, live-verified in logs.
- TRT engine cache was lost with the image rebuild — first controller start
  recompiles (~8-10 min). Re-bake cache into image (stopped commit) after
  next successful start.

### Untethered operation notes (agreed with operator, not yet executed)
- Factory lie→stand→lock: safe untethered; L2+B works there. Proper shutdown
  = factory crouch/lie then power off (no drop).
- Debug mode untethered: ONLY after a wired damp drill (robot lying down).
  Planned stops = spotter grips handle → software damp → manual lower.
- Fall knocked out ethernet AND the Orin AP dies with robot power —
  Wi-Fi-only killswitch is a last resort, keep the gigabit wire draped.

### Footage (~/Videos/g1-demos/)
- `20260810_1547*` (1.2+2.2 GB): run 1, velocity mode — GOOD.
- `20260810_1700*` (1.3 GB + 970 MB): run 2 attempt incl. the fall @ ~17:10
  (~10 min in) — keep for incident review.
- Laptop disk was 100% full mid-session (broke edits silently); now 9 GB
  free after deleting the local image tar (robot has its own copy).

---

# HANDOFF — 2026-08-07 end of day

**One-line status: the G1 is fully deployed and safety-tested; the ONLY thing
between us and the first live teleop session is the headset↔robot Wi-Fi link.**

---

## ✅ What is DONE and verified

### Robot (Orin, 192.168.123.164 — ssh key auth works, sudo pass: try `123`)
- HoloMotion v1.4.0 image **loaded** on the Orin (1.7 TB free disk).
- **Local patches overlaid into the container** and verified present:
  `reference_guard.py` (impossible-pose failsafe), R_y180 parity fix (5
  markers in `_engine_impl.py`), guard wiring in `online.py`, tuned IK
  weights (wrist rot 11→5, elbow pos 7→9).
- Gate check: **"HoloMotion Docker check PASSED. No robot action was sent."**
  (TensorRT + CUDA providers OK.)
- **Kill switch PROVEN on hardware**: `scripts/estop_console.sh` (ENTER=damp
  over wired Ethernet) — leg torque 21.8 Nm → 0.8 Nm on press, robot went
  limp, recovered by remote. Same call as remote L2+B.
- **Watchdog verified against the real robot**: reads LowState @50 Hz over
  Ethernet — tau_est, winding temps (35–59 °C at idle; left ankle runs
  warmest), bus 46 V. CSV logging works. `--on-alarm` auto-damp chain tested.

### Rehearsal (laptop, CPU) — operator-validated
- Full pipeline ran live: PICO 91 FPS → CPU retarget 50 Hz/5 ms → MuJoCo.
- Verdicts: arms/squat/waist/step GOOD after IK tuning; standing lean FIXED
  (waist puck must sit flat on the sacrum); **shrug = impossible (no
  clavicle joint) — closed as hardware**.
- Reference guard: 8/8 unit tests, ZERO false trips during real movement.
- Self-healing supervisor (`scripts/run_rehearsal_supervised.sh`): node
  auto-restarts on stream stall; viewer starts once (no window flapping).

### Tooling ready
- `scripts/deploy_to_robot.sh` — one-command redeploy (already run once).
- `scripts/session_snap.py` — dual camera snap (operator /dev/video6, robot
  /dev/video2).
- Wireless adb to headset: `adb connect <headset-ip>:5555` (survives only
  while headset stays on the same network as the laptop).
- Cloud finetune package staged: checkpoint (model_14000.pt + actor/critic)
  downloaded, shoulder/hip reward config written & hydra-validated
  (`motion_tracking_v1_4_0_finetune_shoulderhip.yaml`).

---

## 🔴 THE BLOCKER — headset↔robot network (full history so nobody repeats it)

The PICO must reach the XRoboToolkit PC service **on the robot** (port 63901,
manual IP entry in the app works — `Enter` button). Everything tried:

| Network | Result | Cause |
|---|---|---|
| Office "VNG Internet" | ✗ forever | **client isolation** — devices get internet but can't reach each other (proven by bidirectional ping fail) |
| Laptop AP (`g1-teleop`, TP-Link RTL8822BU dongle) | ✗ | headset sees SSID but association never reaches the AP (driver-level; tried 5 GHz, 2.4 GHz ch6, pure-WPA2) |
| iPhone hotspot ("Tran's iPhone" / SSID `iPhone`) | ✓ *worked for the whole rehearsal*, then ✗✗✗ | hotspot Wi-Fi **goes dormant** when phone screen off → headset flees to VNG; Orin join failed with "Secrets were required" (WPA3/password mismatch — verify the password shown on the phone's Personal Hotspot screen!) |
| **Robot-hosted AP (NEXT TO TRY)** | untested | `sudo nmcli device wifi hotspot ifname wlan0 ssid g1-teleop password teleop12345` on the Orin; headset joins it, PC Service = `10.42.0.1`. Orin Wi-Fi chip ≠ laptop's flaky dongle, and headset→robot is then the only wireless hop |

Fallbacks if the robot AP also fails to associate: (a) flip Orin AP to
2.4 GHz: add `band bg channel 6` via `nmcli con modify`; (b) buy the damn
dedicated router (runbook's original recommendation — best long-term for a
50 Hz control path anyway).

Wired paths are all fine and independent of this: laptop↔robot Ethernet
(192.168.123.2 ↔ .164, 0.3 ms) carries ssh, e-stop, watchdog.

---

## 📋 TODO next session (in order)

1. **Robot AP**: run the hotspot command on the Orin, verify headset joins,
   PC Service = 10.42.0.1 → WORKING → Send ✓ → Mode Full-body.
2. **First live session** — the ladder (all pre-staged, see SETUP_STATUS §
   deployment steps): hands OFF, gantry snug, spotter with remote, L2+R2,
   estop_console (T1) + watchdog with --on-alarm auto-damp (T2),
   `holomotion teleop` in container (T3). A → feet down → neutral → B.
   ONE test per run: weight shift → slow arms → shallow squat → step.
3. Verify Select-button e-stop behavior in the container FSM (documented,
   not yet source-verified).
4. Known papercuts to fix eventually:
   - App's `Mode` resets to None on every reconnect (retrain user habit or
     find a config persistence option in a newer XRoboToolkit build).
   - Ankle crosstalk = PICO 3-tracker sensing limit (behavioral mitigation).
   - sim2sim baseline eval blocked on missing mesh variant
     (`waist_yaw_link.STL` etc. for `g1_29dof.xml` — the repo's
     thirdparties/HoloMotion_assets pack was never fetched; get it or point
     scene at `g1_29dof_rev_1_0.xml` mesh set).
5. **Cloud finetune** (shoulder/hip fidelity): user registers at
   amass.is.tue.mpg.de → build HDF5 dataset via HoloMotion retarget pipeline
   → rent 4090/A100 → run `finetune_motion_tracking_v1_4_0.sh` with
   `CONFIG_NAME=motion_tracking_v1_4_0_finetune_shoulderhip` (<1 day,
   ~$10–50). Get a per-joint sim2sim baseline BEFORE spending (see #4).

## Key facts that keep biting
- ONE SDK client at a time on the PC service (a second cancels the stream).
- Headset off-head = body stream stops (proximity sensor). Supervisor heals
  the laptop side; on the robot, HoloMotion holds last reference (guard +
  spotter cover it).
- Damp = collapse. Gantry rule until stepping is proven.
- All `.orig` backups sit next to every patched file in
  `HoloMotion/holoretarget/` and `.../target_configs/`.
