# G1 PICO Full-Body Teleop — Handoff & Research Brief
**Date: 2026-08-12 (night). Single source of truth. Prior layers: HANDOFF-archive-2026-08-11.md, HANDOFF-archive-2026-08-12.md.**

---

## 1. TL;DR

We teleoperate a Unitree G1 humanoid with a PICO 4 headset + 2 ankle
trackers through HoloMotion v1.4.0. The robot itself is **proven healthy**
(a bundled dance clip executes flawlessly, twice in one day, telemetry
shows clean commands). Everything that has ever gone wrong lives in the
**reference/retargeting middleware** — the chain that turns human motion
into a 50 Hz reference stream — and in one **stock configuration hazard**
(limit scaling) that turns dirty references into violent joint chatter.
HoloMotion is now parked; we are migrating the whole middleware to NVIDIA
GEAR-SONIC (GR00T-WholeBodyControl) while keeping every lesson documented
here. The user's research question: **why does the G1 keep malfunctioning
in the retargeting middleware state?** Sections 3–5 are the evidence base
for that question.

---

## 2. Exact stack

**Hardware**
- Unitree G1 EDU Ultimate, 29 DoF, Inspire FTP hands (unused in this work).
- PC2 on robot: Jetson Orin NX, JetPack 5.1, `unitree@192.168.123.164`.
- Operator: PICO 4 headset + 2 PICO Motion Trackers (ankles). Waist
  tracker owned but deliberately excluded (see §4).
- Laptop: ThinkPad, Ubuntu 22.04, Intel Core Ultra 5 225H, 22 GB RAM,
  **no NVIDIA GPU**. Wired to robot as `192.168.123.2` (USB-GbE adapter
  `enx000ec6c3d44a`, NM connection `robot-lan-usb`).
- Safety rig: gantry/tether + human spotter, mandatory at Start.

**Software (HoloMotion track — parked but fully functional to study)**
- HoloMotion v1.4.0, Docker image
  `horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64` on the Orin.
  - `humanoid_control` C++ node: FSM (ZERO_TORQUE → MOVE_TO_DEFAULT →
    velocity/tracking), PD control @ 500 Hz. **Deployed binary = STOCK
    Jul-16 build** (md5 5764c810…, 5,605,520 B). The shipped source does
    NOT rebuild into a working binary (08-11 finding) — the modified
    `src/src/main_node.cpp` in this repo is *not* what runs.
  - `policy_node_29dof` Python: ONNX policies (velocity + motion tracking
    v1.4.0, TensorRT EP), consumes the reference stream.
- Laptop retarget node: `HoloMotion/deployment/holomotion_teleop/
  holomotion_teleop_node.py` — XRoboToolkit SDK client → 24-joint body
  poses @ ~90 Hz → Newton-based IK (holoretarget, CPU ~5 ms/solve) →
  `reference_qpos` (36-dim) → ZMQ PUB @ 50 Hz, `tcp://*:6001`.
- Robot subscriber: ZMQ SUB (CONFLATE=1) → ReferenceBuffer (20-deep) →
  `_poll_zmq_reference()` → VrReference queue → motion-tracking policy.
- XRoboToolkit PC service (`holosim-pcservice`, port 63901) on the laptop;
  the PICO app connects to it and streams tracking.
- Launch profile: `launch_profiles/orin_docker.yaml` — `reference_source:
  zmq`, `motion_observation_backend: cpu`, `max_data_age: 1.5`,
  `build_before_launch: false` (all deliberate local fixes; see archive).

**Software (SONIC track — the migration target)**
- NVIDIA GEAR-SONIC / GR00T-WholeBodyControl at `~/GR00T-WholeBodyControl`.
- Laptop: `.venv_teleop` (CPU torch 2.13, mujoco 3.11) — verified.
  Launchers: `scripts/sonic_sim.sh` (MuJoCo sim, **must run with
  `--interface lo` + `env -u CYCLONEDDS_HOME -u LD_LIBRARY_PATH`**, both
  now baked in) and `scripts/sonic_teleop.sh` (PICO streamer; **never**
  use `--vis_smpl/--vis_vr3pt` on this GPU-less laptop — hard C++ crash).
- Robot side requires JetPack 6.2 reflash (TensorRT 10.7 EXACTLY; wrong
  TRT = silently wrong actions). Flash image verified on laptop; the 9.94G
  `Jetpack_6.2_nx.tar.bz2` is quota-blocked by Google Drive at 667M with
  an hourly auto-resume loop running (log:
  `/media/alois/SONIC-FLASH/gdown.log`).

**Network topologies (a core part of the problem — see §3.4)**
1. Robot-AP (used most of the project): headset → robot-hosted AP
   `g1-teleop` (10.42.0.1) → NAT on Orin → laptop `192.168.123.2:63901`.
2. Phone-hotspot (the topology of the user's first-ever good run; being
   restored): headset + laptop both on iPhone hotspot **Wi-Fi**; app →
   laptop's hotspot IP. **The USB-tether leg does NOT work** for body
   data (Apple isolates USB↔Wi-Fi legs for the streaming traffic; head
   pose/TCP passes, body stream never arrives).
3. Corporate Wi-Fi: never as transport (assumed client isolation); its
   main role in this saga was as a **hijacker** — the headset auto-hopped
   to it mid-session because `g1-teleop` has no internet.

---

## 3. THE PROBLEM: retargeting-middleware malfunction (research focus)

The pipeline, end to end:

```
PICO 4 (90 Hz inside-out tracking, 2 ankle trackers, skeletal solve)
  → Wi-Fi transport (one of the topologies above)
  → XRoboToolkit PC service (63901, ONE SDK client slot)
  → SDK client in laptop retarget node (PicoReader thread)
  → Newton IK retarget (~5 ms) → 50 Hz reference_qpos
  → ZMQ PUB/SUB over the wire → ReferenceBuffer on robot
  → policy_node reference intake (staleness, readiness, guards)
  → motion-tracking ONNX policy → joint targets → C++ PD @ kp≈300
```

Every failure we have measured, by layer:

### 3.1 Reference discontinuities (the original "violence")
- Measured reference teleports up to **1.34 m root / 0.71 rad joint in a
  single frame** on fresh (non-stale) frames (08-11, live VR).
- Saved tapes show **cross-restart joint discontinuities of 0.60, 2.17,
  1.16 rad** when the laptop node restarts mid-session (external safety
  review of `~/ref_tapes/*.npz`).
- Steady-state stream is CLEAN: p95 joint step 0.017–0.05 rad, root p95
  3–6 mm, measured over 23k frames. **The violence is exclusively a
  transition-window phenomenon** — node restarts, stream resumes, stale
  recoveries — never steady state.

### 3.2 Transition-window anatomy (measured live, 08-12)
- Laptop node bounce ⇒ ~10 s robot-side stale gap ⇒ ZMQ auto-reconnect ⇒
  frame_index regresses (measured 14000 → ~200) ⇒ without guards the old
  and new streams splice = reference teleport at full policy authority.
- "Frozen-fresh": a wedged upstream re-sends one pose at 50 Hz with fresh
  timestamps — an afk tape shows an **84-frame (1.7 s) bit-identical run**.
  The robot tracks a frozen ghost, then jumps on recovery.
- The robot's `reference_arrival=50.1Hz` log stat is a **fossil** during
  these gaps (computed from the last 20 stored stamps) — it looks alive
  while nothing arrives. Do not trust it as liveness.

### 3.3 SDK/session-layer wedges
- XRoboToolkit's `BodyDataAvailable` is a **write-once latch**
  (py_bindings.cpp): after headset sleep/disconnect it stays true while
  timestamps freeze — the node spins on stale data forever, silently.
- The PC service allows **ONE SDK client**; an uncleanly killed client
  (SIGKILL) leaks the slot and the next client connects deaf. All
  laptop-side kill paths must be SIGTERM-first (the node saves its tape
  and calls xrt.close() on TERM).
- Half-open TCP: the service shows an ESTABLISHED connection with zero
  data after the headset vanishes from the network — "connected but dead"
  is the wedge signature.

### 3.4 Transport-layer causes of the above
These are what actually *trigger* most wedges/transitions:
- **Headset proximity-sensor sleep**: lift the headset off your face to
  watch the robot → display sleeps → tracking and stream stop instantly.
  Rule: headset stays worn from Send-tick to damp; spotter watches robot.
- **Corporate Wi-Fi auto-hop**: `g1-teleop` has no internet, so PICO OS
  "upgrades" to a remembered internet SSID mid-session. Fix: forget it.
- **App IP field auto-reverts to 10.42.x** on every boot; sessions
  silently point at a dead endpoint until re-typed (`192.168.123.2`, or
  the laptop's hotspot IP in topology 2).
- **Apple USB-tether leg isolation** (found tonight): body stream never
  crosses USB↔Wi-Fi inside the iPhone; both peers must be on the Wi-Fi.
- One loose Ethernet cable event mimicked all of the above at once.

### 3.5 The amplifier: stock configuration hazard (external safety review)
Even a *clean* reference is executed dangerously by the stock deployment:
- `src/config/g1_29dof_holomotion.yaml` ships **position/velocity/effort
  `limit_scales: 2.0`** — commands may go to **double** the nominal joint
  range — and C++ POLICY mode **bypasses torque limiting**.
- Measured consequence (telemetry, A-stand): ankle commanded **+1.222 rad
  vs +0.524 rad physical max** at kp≈289; the joint physically can't
  follow; it limit-cycles at 1–3°/tick. That chatter IS the visible
  "microstepping/twitching", it heated `waist_pitch` to ~130 °C with a
  motor fault (state 512, cleared by power cycle), and it is why the
  custom stack feels nothing like Unitree's native controller.
- Command stream itself is **clean** (max per-tick command step 0.02 rad)
  — the violence is command *magnitude vs limits*, not command noise.
- Additional review findings: the C++ A-transition safety gate is a no-op
  (joint-name mismatch), A has an unblended entry path, NaNs are logged
  but still published. None of these are fixed — HoloMotion is parked.

### 3.6 The deployment trap that hid everything (meta-finding)
ros2 launch executes an **extensionless full copy** of
`policy_node_29dof` at `install/humanoid_control/lib/humanoid_control/`
directly as `__main__`. Every overlay loop matching `-name '*.py'` missed
it — **weeks of policy-node-level patches never actually ran** until
2026-08-12. Module-level patches (`local_retarget.py`,
`policy_runtime.py`) always ran. If a patch "mysteriously doesn't fire":
check `tr '\0' ' ' </proc/PID/cmdline` and pyc regeneration times.

---

## 4. Eliminated hypotheses (each with its proof)

| Hypothesis | Verdict | Proof |
|---|---|---|
| Robot hardware/gears broken | **Eliminated** | Offline bundled clip flawless 2× in one day; hand check of ankles normal; telemetry "errors" explained by out-of-range commands + PD gravity droop (err = τ/kp ≈ 0.1–0.2 rad at ankle, inherent to MOVE_TO_DEFAULT) |
| IMU broken (post-falls) | Eliminated | Projected gravity −1.000, gyro ~0 stationary; the ~6° stand tilt tracks the command/limit story, gyro quiet |
| Waist tracker perturbs skeletal solve | Not the cause | 2-tracker run still twitched (08-12 AM) |
| Tuned IK weights | Eliminated | A/B tuned vs stock both measured clean; stock in place (`smplx_to_g1.json`, tuned saved as `.tuned-0812`) |
| Tether/gantry tension | Eliminated | Same tether during flawless clip runs |
| Warp GPU JIT freeze (08-10 fall) | Fixed | Root-caused + reproduced in sim; `motion_observation_backend: cpu` |
| Factory controller interference | Controlled | `release_factory_control.sh` gated in bringup; verify `name:''` |
| "Gains were reverted" theories | Eliminated | git-verified: no gain/config delta vs upstream except orin_docker.yaml infra keys |
| Steady-state stream dirty | Eliminated | 23k-frame tape analysis, p95 ≤0.05 rad; 12 s passive probe clean |

---

## 5. Current best understanding (one paragraph)

The G1 "malfunctions in the retargeting middleware state" because the
middleware sits on a **fragile session/transport chain** (one-client SDK
latch, sleep-prone headset, network auto-hopping, IP-reverting app) that
produces **transition windows** — restarts, stale gaps, frozen streams —
whose recovery splices discontinuous references; and because the stock
executor **amplifies** any imperfect reference by allowing commands to
2× joint limits with no torque cap, turning even modest reference error
into limit-cycle chatter that looks like violent twitching and cooks
motors. The robot below this chain is provably fine. Fixing this class of
problem requires (a) transport that cannot silently die (or guards that
provably catch every transition), and (b) an executor that clamps to
nominal limits with torque limiting — or replacing the middleware
entirely, which is what the SONIC migration does.

---

## 6. Mitigations deployed on HoloMotion (state: live, v1)

- Laptop output step guard (clamps per-frame reference steps; visible in
  tapes as exact 0.3000 rad ceilings doing their job).
- Robot-side transition guards in the *actually executed* entry copy:
  frame_index regression → VR readiness reset; frozen-fresh (≥25
  identical frames) → withhold so the stale fallback engages.
  Known v1 holes (safety review): queue not purged on regression;
  first-frame-after-restart bypasses the publisher step guard.
- Honest freeze detection in PicoReader (only reports stamps frozen
  >0.25 s — the 08-11 detector false-alarmed every 5 s on healthy
  streams and polluted a full day of diagnosis).
- Supervisor: SIGTERM-first bounces (tape saved + SDK slot freed), one
  reference tape per node incarnation in `~/ref_tapes/`.
- Deploy tooling patched for the five-copy trap (`robot_install.sh`
  overlays extensionless entry copies too).
- E-stop truth: `estop_console.sh` (LocoClient.Damp) is a **false
  positive** in released/debug mode; `estop_console2.sh` (docker kill →
  firmware damp) requires `CYCLONEDDS_HOME=~/.local/cyclonedds` or it
  throws at damp time. **The REMOTE is the primary safety path**
  (Y = downshift, Select = damp+kill); laptop SDK stops are best-effort.
- `scripts/ready_loop.sh` exists but **must stay off** — an auto-restart
  loop defeats latched safety stops (review finding; agreed).

---

## 7. Open research questions

1. Why does the PICO/XRoboToolkit skeletal solve emit metre-scale root
   teleports on tracking recovery instead of flagging invalid? Is there a
   confidence field we never consumed?
2. Can the one-client PC service be made restart-transparent (session
   tokens? keepalive probing from the client side)?
3. `BodyDataAvailable` write-once latch: upstream bug report worth filing
   (py_bindings.cpp) — reproducible on every headset sleep.
4. What is the *stock* HoloMotion rationale for `limit_scales: 2.0` and
   POLICY-mode torque bypass? (Sim-trained headroom that is unsafe on
   hardware?) Does upstream Horizon deployment really run this?
5. The MOVE_TO_DEFAULT no-ramp wedge (commands frozen at endpoint, kp 350,
   firmware not executing, cleared only by power cycle): firmware damp
   latch? Reproduce with lowcmd capture on next opportunity.
6. Does SONIC's reference path (50 Hz retarget on laptop, its own
   transport) exhibit the same transition-window class? Design the
   bring-up tests to probe restarts/sleeps FIRST, not last.

---

## 8. SONIC migration state (the replacement for this middleware)

- Laptop env done + verified; sim runs (`--interface lo` fix baked in;
  `sudo ip link set lo multicast on` needed after every laptop reboot —
  not yet persistent).
- Sim-teleop (headset → MuJoCo) blocked only by transport tonight: use
  phone-hotspot **Wi-Fi** for both peers (USB leg drops body data), or
  wait for robot AP. MuJoCo window must be launched from the user's own
  desktop session (background/service contexts cannot own a window).
- Robot: waiting on JetPack 6.2 package (quota-blocked download,
  auto-retrying hourly) + FLASH DAY per `MIGRATION-SONIC.md`.
- **Hardware gate before first SONIC run**: inspect waist + ankle
  actuators (the 130 °C/512 event), then gantry qualification ladder.
- All irreplaceables backed up in `~/orin-reflash-backup/`; robot SSD is
  safe to wipe.

---

## 9. Operational reference (the hard-won facts)

- **Remote under custom control**: Y = safe downshift to velocity stand;
  Select = damp then motors off (support first); L1 alone historically =
  free-fall trap with stock binary; A only from MOVE_TO_DEFAULT; Start
  only from ZERO_TORQUE (every damp kills the app → app must be restarted
  before Start works — this cost us ~6 round-trips in one day).
- **Power-cycle discipline**: never mix factory running-mode and custom
  control in one cycle; full cycle → damp → release → verify `name:''`.
- **Remote dead?** `rt/wirelesscontroller` DDS topic silent = remote
  off/unpaired/battery — it normally streams continuously.
- **Bringup**: `scripts/session_up.sh` (idempotent). Offline clip A/B:
  `scripts/holomotion_offline_test.sh on|off` (container-local toggle —
  re-apply after any container recreation).
- **Networks**: laptop must NEVER join `g1-teleop`; headset app IP field
  reverts to 10.42.x on boot — retype every session.
- pkill patterns self-match your own shell — kill by PID list.

## 10. Artifact index (local, not in repo)

- Reference tapes: `~/ref_tapes/ref_YYYYMMDD_HHMMSS.npz` (one per node
  incarnation; keys: frame_index, pico_body_poses(24,7), reference_qpos(36)).
- Command-vs-state telemetry: `~/.claude/jobs/1716853a/tmp/*.csv`
  (lowcmd targets + kp vs lowstate q @ 500 Hz; the stuck-ramp and
  A-stand captures that produced §3.5 numbers).
- Watchdog CSV (torque/temp/motorstate): `/tmp/session_monitor/
  watchdog_151420.csv` (the 130 °C / state-512 evidence).
- Session camera tapes: `~/Videos/g1-demos/2026-08-12*.mp4`.
- Audits: `AUDIT-2026-08-11.md`, `AUDIT-2026-08-11-night-report.md`
  (40-agent audit, 27 findings), external safety review 2026-08-12
  (triaged in HANDOFF-archive-2026-08-12.md).
