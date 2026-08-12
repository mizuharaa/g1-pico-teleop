# HANDOFF — 2026-08-11 MIGRATION TO SONIC STARTED (top priority thread)
User decision: migrate to NVIDIA GR00T-WholeBodyControl (GEAR-SONIC) —
native PICO-4 whole-body teleop with jump/squat/kneel/run modes. This
SUPERSEDES the HoloMotion blocker backlog (kept below for the record).
**Read MIGRATION-SONIC.md** — laptop side DONE and verified; ONE gate remains: Orin JetPack 6.2 reflash (physical,
user-approved, back up first). Hardware CLEARED via running test —
the knee-damage theory is retracted (confounded torque evidence).
Old stack: frozen + fallback-ready (robot image untouched; holosim units
disabled except pcservice, which SONIC reuses).

# HANDOFF — 2026-08-11 CONSOLIDATED (one section, supersedes the whole day)
Full audit (40 agents, 27 confirmed findings): AUDIT-2026-08-11-night-report.md

## VERDICT — why the robot shakes
A) TRACKING violence: the reference stream teleports (measured 1.34 m root /
   0.71 rad joint steps in fresh 20 ms frames; wrist pinned at limit 135
   frames). Causes: PICO tracking loss in the robot room AND self-inflicted
   node-restart pose blending (no frame_index regression check; sub-0.6 s
   restarts splice pre/post poses). NO output-side step guard exists.
B) STAND chatter: knees/ankles hammer (65-78 Nm single-tick steps, wrists
   0.03) with NO reference involved. Timeline: smooth run happened BEFORE
   the 08-10 fall; all chatter after. Plus waist_pitch THERMALLY TRIPPED
   twice on 08-10 (130°C, error 512). Hardware inspection = hard gate:
   knees, ankles, waist actuator.
C) OPEN question (corrected claim): commanded targets are NOT proven smooth
   in shake windows — targets_rec.csv covered 15:44-15:46 (wrong window),
   and instrumented data shows 0.33 rad/20 ms target steps in QUIET STANCE.
   The policy-output smoothing question must be settled in sim (gating
   thresholds) before hardware.

## THE FIVE PEELED TRAPS (all real, all handled)
1. Factory 'ai' controller never releases via L2+R2 chord — fought our
   controller all day. FIX: release_factory_control.sh, now a hard gate in
   session_up.sh (verifies name:''). Re-check after EVERY power cycle.
2. humanoid_control CANNOT be rebuilt from shipped source (3 rebuilds, 3
   failure modes: -O0 slow loop; -O2 CRC-dead via strict aliasing; -O2+
   no-strict-aliasing misbehaved). Pristine binary RESTORED (md5
   5764c810ff34); experiment preserved in image holomotion:patched-backup-
   0811. ⚠ CONSEQUENCE: bare L1 = INSTANT FREE-FALL (L1 patch was C++).
3. g1-dance killswitch fires on L2+B incl. during setup chords: damps +
   docker-kills the app. Disarm during setup, arm for sessions.
4. A-button re-enable: every A press snaps targets to default unblended
   (8x in 5.8 s logged during shaking). Press A once. Fix queued (below).
5. limit_scales 2.0 doubles position clamp range + POLICY state has NO
   torque limiting. Tighten before next tracking run.

## CURRENT DEPLOYED STATE
Robot image = all 08-10/08-11 python+config patches (soft-start, both mode
blends, slew knob, zmq+cpu source pair, max_data_age 1.5, build_before_
launch false) + STOCK C++ binary + baked TRT cache (boot ~50 s). Slew IS
ACTIVE on robot (start_teleop_container.sh -e HOLOMOTION_TARGET_SLEW_RAD_S
=15). Laptop: systemd units holosim-pcservice / holosim-chain / holosim
(sim stopped during robot sessions), ref smoothing env on chain unit.
Topology: headset->g1-teleop AP->robot NAT->laptop 192.168.123.2 (PC svc)
->retarget node->zmq :6001->robot policy AND/OR sim. NEVER join laptop to
g1-teleop. Footage: ~/Videos/g1-demos + robot:~/footage_archive.

## RANKED BLOCKERS BEFORE NEXT HARDWARE RUN (audit §1, do in order)
1. A-snap guard: policy_runtime.py:164 — guard A edge with `not
   policy_enabled` (or route to switch_to_velocity_mode in motion mode);
   add blend capture + stale-blend clear in enable_velocity_policy;
   note reset_counters clears slew anchor (first tick unclamped).
2. NaN chain: _publish_action_target must hold-last-good + e-stop (now
   logs and PUBLISHES NaN); isfinite gate in main_node PolicyActionHandler
   (python-side wrapper since cpp is unbuildable — clamp in policy node
   before publish); fix slew NaN poisoning; remote_controller_filter: add
   positive vx clamp, isfinite on 4 floats, NaN-proof EMA.
3. Frozen-fresh stream: content/frame_index progress staleness in the zmq
   buffer; output step clamp (root 0.15 m / joint 0.3 rad per frame) or
   resume ramp; frame_index regression detection in vr_reference.store().
4. Supervisor 5-liner: ss awk $4 (bounce NEVER fired — column bug),
   graceful kill (kill -9 leaks the single SDK slot), flock fd leak
   (200>&- on spawned children), SEEN_DATA re-arm on head_pose_ok,
   init_xrt SIGTERM handling + xrt.close().
5. limit_scales 2.0 -> 1.0 in g1_29dof_holomotion.yaml (redeploy).
6. zmq subscriber immortality: try/except per packet in recv loop.
7. Telemetry first: launch_runtime --record + export bag (raw
   wireless_remote in lowstate mcap = forensics for everything).
Sim additions to gate all of it: audit §3 (factory-controller rig, CRC/SIL
gates, encoder noise, latency model, new scenarios incl. A-mid-motion,
NaN frame, frozen-fresh, frame_index regression, B with large offset;
promote max_target_step + stance tau-ripple to GATING).

## NEXT SESSION ORDER
1. HEX KEY: hand back-drive knees/ankles L/R + waist; any notch = open it.
2. Blockers 1-7 (code, laptop-only) + sim gates green.
3. PICO room: lock tracking 50 Hz, bright non-reflective light, clean IR
   lenses, move AP to 5 GHz.
4. Only then hardware, with --record on, killswitch armed post-setup,
   spotter ladder (Y first / Select supported / NO bare L1 / one A press).

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
