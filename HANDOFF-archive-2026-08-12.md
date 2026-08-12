# HANDOFF — 2026-08-12 (single source of truth; prior day archived in HANDOFF-archive-2026-08-11.md)

## 2026-08-12 SAFETY VERDICT TRIAGE (external review, accepted with caveats)
ROOT MECHANISM (supersedes gear-damage AND droop-only theories): stock
config ships limit_scales 2.0 + POLICY-mode torque-limit bypass → policy
commands beyond physical joint range (ankle cmd +1.222 vs +0.524 limit) →
firmware clamps → limit-cycle chatter = the microstepping/twitch/heat and
the un-native gait feel. "All stock" ≠ safe — stock IS the hazard.
- waist_pitch hit motorstate 512 at ~130°C during the clip test (watchdog
  CSV); CLEARED after power cycle (verified all-zero, cool) but the motor
  deserves physical inspection BEFORE first SONIC hardware run (same motors).
- ready_loop.sh = safety hole (auto-resurrects the app past any latched
  stop) — STOPPED permanently; do not revive for SONIC.
- Guards v1 incomplete: regression reset doesn't purge queue; step guard
  passes first post-restart frame; A-path unblended. Moot for HoloMotion
  (parked) but design lessons for SONIC bring-up.
- Caveat on the review: C++ line cites refer to the MODIFIED source, not
  the deployed stock binary; behavioral CSV evidence stands regardless.
- simsmoke regression: 0/8 scenarios runnable (missing extracted model
  config, bench/simsmoke/sim_port.py:22) — broke at some point today.

## 2026-08-12 FINAL (supersedes the earlier end-of-day note) — HOLOMOTION
## PARKED FOR GOOD AFTER LAST LIVE ATTEMPT; SONIC IS THE PATH
- Last live-VR attempt still twitched despite: clean 30s-verified stream,
  all guards live, corporate-wifi forgotten, correct IP, remote replaced
  (batteries), fresh power cycle. User called it: swap to SONIC.
- Facts that survive into SONIC: robot mechanics/policy/PD PROVEN GOOD
  (offline clip flawless 2x same day, telemetry: clean commands, droop
  normal); remote channel (rt/wirelesscontroller) silent = dead remote
  battery/pairing (diagnosed once); headset proximity-sensor sleep kills
  the stream when lifted off the face (wear it or tape the sensor);
  PICO app IP field auto-reverts to 10.42.x — must read 192.168.123.2;
  MOVE_TO_DEFAULT can wedge with cmd frozen at target + firmware not
  executing (danger: 68°-error@kp350 pending — damp before it unlatches;
  full power cycle clears it).
- All laptop HoloMotion machinery STOPPED (supervisor, node, ready_loop,
  log tails). Robot app still in the image on the Orin — irrelevant, the
  Orin gets reflashed for SONIC.
- SONIC: JetPack download QUOTA-LIMITED by Google Drive at 667M/9.94G
  (file id 1bcED2Vy64fyOWIBxK9ck0iXA_Lo9TOyg; "many accesses" error).
  Hourly auto-retry loop running overnight (resumes via gdown --continue;
  log: /media/alois/SONIC-FLASH/gdown.log). If still quota-blocked next
  session: log into Google in a browser, add file shortcut to own Drive,
  download from there. NOTE: iPhone USB tether hijacked the default route
  for ~1h (metric 101) — part of the download rode the phone plan; USB
  now unplugged, corporate wifi restored as the only internet route.
- SIM TELEOP STATUS (2026-08-12 night): MuJoCo window WORKS from the
  user's own terminal (background/headless launches can't own a window).
  Head/control link works over the iPhone hotspot, but BODY DATA never
  flows when the laptop is on the USB-tether leg — Apple partially
  isolates USB↔WiFi legs; the working recipe is BOTH laptop and headset
  on the hotspot WIFI (single wifi radio conflict with corporate = do it
  after the download finishes, or during robot sessions where internet
  doesn't matter). Corp-wifi-as-transport untested (user expects client
  isolation). Robot battery charging overnight.
- NETWORK TOPOLOGY CHANGE (2026-08-12 evening, user-driven): iPhone hotspot
  replaces the robot AP for the headset link. Headset + laptop both on the
  hotspot (laptop via USB-C tether = enxea78656ce38f 172.20.10.2); app
  points at 172.20.10.2. Robot AP no longer needed for headset transport —
  robot only gets the wired zmq feed. Kills: corporate-wifi auto-hop,
  10.42-vs-192 IP confusion, robot-must-be-on-for-headset dependency.
  User's first-ever OK run was on a phone hotspot — this restores that.
- SONIC SIM FIXES (2026-08-12 evening): sonic_sim.sh now runs with
  --interface lo (fixes "[ChannelFactory] create domain error" AND
  guarantees sim DDS never reaches the robot wire) + env -u CYCLONEDDS_HOME
  -u LD_LIBRARY_PATH (old ~/robot stack leaks via login env). Loopback
  needs `sudo ip link set lo multicast on` — DONE today but NOT persistent
  across laptop reboots (add to bring-up if sim DDS fails after reboot).
  pico_manager --vis_smpl/--vis_vr3pt CRASHES the manager on this no-GPU
  laptop (C++ terminate shortly after engage) — run WITHOUT vis flags.
  A+B+X+Y is engage AND e-stop: one short press only.
- User verdict mid-session: "holomotion looks hopeless" → full pivot to the
  SONIC migration. JetPack tar downloading to the SONIC-FLASH USB stick
  (ETA ~7-8h from ~15:45); FLASH DAY checklist is next (MIGRATION-SONIC.md).
- HoloMotion state PARKED here: robot damped+safe, app down. Unfinished:
  offline-clip rerun (was armed, FSM wedged in MOVE_TO_DEFAULT holding
  stiff without ramping — unexplained; cleared by damp), lean/twitch cause
  still open (software fully exonerated by the audit below; physical
  calibration/gear check never performed — user disputes hardware theory).
- ⚠ TWO SAFETY BUGS FOUND AT THE WORST MOMENT:
  (1) estop_console2/g1_estop.py FAILS without CYCLONEDDS_HOME=~/.local/
  cyclonedds in the environment (CycloneDDSLoaderException at damp time —
  the wired kill was DEAD when the user needed it; Claude damped via ssh
  docker-kill + g1_estop with env set). FIX BEFORE ANY FUTURE RUN: export
  the env inside estop_console2.sh/arm scripts, and test-fire at bringup.
  (2) The offline-test toggle lives in the CONTAINER fs — every docker
  kill/recreate reverts it; re-apply after any e-stop.
- If HoloMotion is ever resumed: read the EVENING audit section (software
  byte-verified stock; 6° measured stand tilt; next step was mechanical
  inspection + joint recal + offline clip rerun).

## 2026-08-12 EVENING — FULL SINCE-08-10 AUDIT: SOFTWARE EXONERATED FOR THE
## LEANING/TWITCHY STAND; VERDICT = PHYSICAL (calibration or gears)
User-demanded stuff-by-stuff comparison vs the successful 08-10 run. Result,
every layer byte-verified (md5 across current image, backup-0811 image, git):
- C++ humanoid_control binary (FSM/MOVE_TO_DEFAULT/PD): STOCK Jul-16 build
  in the CURRENT image (5,605,520 B, md5 5764c810...). NB the backup-0811
  image holds the known-BAD Aug-11 rebuild (1,147,496 B) — backup predates
  the stock restore. Do NOT "roll back" to backup-0811's binary.
- default pose / kps / kds / limit_scales: ALL STOCK (config.py unmodified
  vs git, no policy-config diffs anywhere in the image).
- Live deltas vs stock v1.4.0 are exactly 4 files: orin_docker.yaml (4 infra
  changes: build_before_launch false, zmq source, max_data_age 1.5, cpu
  backend), local_retarget.py (+27 suppression), policy_runtime.py (+132,
  byte-identical laptop↔image, live during the flawless morning run),
  policy_node entry (+34 input-side guards). NONE touch stand posture,
  gains, or the PD path.
- MEASURED: in "locked standing" (pure PD joint hold, no policy, no VR) the
  body tilts ~6° (projected gravity [+0.018,-0.108,-0.994]), gyro quiet.
  A pure joint-space hold leaning = joint-level position error: ankle/hip
  ZERO-CALIBRATION drift or gear damage from the falls. Memory gate was
  already "INSPECT GEARS first".
- User observation same session: robot leans forward on Start, feels like
  it would fall untethered; velocity stand microsteps/twitches worse than
  before; native factory gait (tested this cycle) felt normal-strong.
NEXT (in order): (1) physical inspection ankles/knees/feet + gear check;
(2) Unitree joint zero recalibration (factory mode / Unitree app, own power
cycle); (3) re-run scripts/holomotion_offline_test.sh — this morning it was
FLAWLESS; if it twitches after today's handling, that confirms physical
drift with zero VR involved; if flawless again, stand test again on ground.
Decisive live-VR run stays queued behind mechanical clearance.

## 2026-08-12 LATE-PM — THE FIVE-COPY TRAP (biggest find of the day)
1. ⚠ ros2 launch executes an EXTENSIONLESS FULL COPY of policy_node_29dof at
   install/humanoid_control/lib/humanoid_control/policy_node_29dof (run
   directly as __main__, never generates a .pyc). Our installer's
   `-name '*.py'` overlay loop NEVER matched it → every robot run to date
   executed the STOCK v1.4.0 policy-node logic; the 08-12 transition guards
   (and any other policy_node-level patch) were riding dead in module
   copies that are imported for classes but whose node logic never runs.
   Found because the frame_index-regression guard stayed silent through a
   provable ~14000→200 regression during a node bounce.
   FIXED: entry copy patched in container + committed to image + app
   restarted (verified setup-complete); robot_install.sh overlay loop now
   also finds extensionless entry copies (STEM match under lib/humanoid_control).
   NOTE: local_retarget.py / policy_runtime.py patches were always live
   (imported as modules) — only policy_node-level logic was stale.
2. FALSE-FROZEN FIX (laptop): the 08-11 PicoReader FROZEN warning fired on
   ANY single unchanged-stamp poll (1 MHz poll vs 90 Hz stream) rate-limited
   to 5 s → a fake "timestamps FROZEN" line every ~5 s on healthy streams,
   polluting operator judgment + re-arming the supervisor's 90 s grace.
   Now only reports a stamp frozen >0.25 s continuously. After the fix:
   0 FROZEN lines on a healthy stream. (Morning's "SDK latch" reads may
   have been partly this artifact.)
3. SUPERVISOR: NEW-app-connection bounce changed kill -9 → graceful_kill
   (tape saves + SDK client slot freed); node now always launched with
   --debug-retarget-dump ~/ref_tapes/ref_<ts>.npz (one tape per node
   incarnation, saved on SIGTERM — mechanism PROVEN, 3 tapes captured).
4. NODE-BOUNCE ANATOMY (measured live): bounce ⇒ ~10 s robot-side stale gap
   ⇒ ZMQ SUB auto-reconnects ⇒ splice risk at recovery instant (fi
   regression) — exactly the transition window; now guarded ON ROBOT for
   real. Robot "reference_arrival=50.1Hz" during stale is a FOSSIL stat
   (computed from last-20 stored stamps; a dead/quiet subscriber keeps
   showing 50 Hz forever) — do not trust it as liveness.
5. TAPE ANALYSIS (2 tapes, 23k frames): steady-state clean (p95 joint step
   0.017–0.05 rad, root p95 3–6 mm); real hops cluster ONLY at transitions
   (headset don/doff, tracking events); max joint step exactly 0.3000 rad in
   both tapes = laptop output step-guard CLAMP doing its job on real
   teleports; afk tape shows an 84-frame (1.7 s) frozen-fresh run sent at
   50 Hz — validates the robot-side FROZEN-FRESH guard (fires at 25).
6. estop footer in session_up.sh corrected to estop_console2.sh.
STATUS AT WRITE: robot ZERO_TORQUE, factory released, patched app up,
supervisor+node running (guard suppressing while operator afk), camera
recorder STOPPED to save disk (restart via record_session.sh start before
next run). Decisive instrumented run NOT yet performed — next power-on/user
return: restart cameras, verify stream clean, ladder Start→A→hold→B on
Claude's call.

## HOLOMOTION FINAL DIAGNOSIS (parallel track, prepped 2026-08-12)
Premise-check done: NO gain/config delta vs upstream except orin_docker.yaml
(git-verified) — "revert your gains" theories are dead. Three tests queued
for next robot power-on:
1. THE BISECTION — scripts/holomotion_offline_test.sh on|off: tracking mode
   with the BUNDLED clip, zero VR. Twitch = policy/hw; clean = reference
   path. Tethered + spotter, ladder Start->A->B.
2. scripts/imu_health_check.py — post-fall IMU sanity (robot stationary):
   projected gravity ~[0,0,-1], gyro ~0. Fail -> Unitree IMU recal first.
3. Next VR run: REMOVE THE WAIST TRACKER (validated config = 2 ankle
   trackers only; waist tracker perturbs PICO's skeletal root solve =
   plausibly our measured reference teleports).
Power-cycle discipline (adopted): never mix factory running-mode tests and
teleop in one power cycle; full cycle -> damp -> release_factory_control
-> verify name:'' -> teleop.

## 2026-08-12 BISECTION RESULT + SAFETY REVELATION (read before any run)
1. OFFLINE CLIP TEST: PERFECT execution ("couldn't have been better").
   Policy, gains, PD, motors, gears, IMU: ALL PROVEN GOOD. The tracking
   violence is 100% the live VR reference path (teleports: waist tracker /
   PICO room / retarget splicing). HoloMotion is one clean-reference away
   from working. Next VR run: 2 ankle trackers ONLY, room hygiene, then
   the output step-guard if still dirty.
2. ⚠ SAFETY: estop_console.sh (LocoClient.Damp) DOES NOT WORK in debug
   mode — the factory service it calls is released; its "delivered" is a
   false positive on timeout. USE scripts/estop_console2.sh (docker kill
   -> firmware damp) as the wired kill. Remote Select button NEVER
   registers in FSM logs (hardware/mapping? investigate); B/A/Start/Y fine.
3. IMU verified healthy post-fall (gravity -1.000, gyro ~0).

## 2026-08-12 PM VERDICT — the transition-window theory (current truth)
- Robot+policy PROVEN good (offline clip flawless). IK weights EXONERATED
  by A/B (tuned vs stock both measured clean; STOCK now in place, tuned
  saved as smplx_to_g1.json.tuned-0812). Waist-tracker removal did NOT fix
  the twitching (user confirmed 2-tracker run still twitched).
- Reference stream measures CLEAN in steady state (p95 step 0.08 rad,
  root 10 mm, zero pinning) yet violent runs keep happening — every one
  coincides with a TRANSITION WINDOW (post-stale recovery, node restart,
  frozen-fresh wedge). Morning's violent run started right after a STALE
  event.
- FIXES IMPLEMENTED (laptop, compiled): policy_node_29dof.py transition
  guards: (1) frame_index regression -> reset VR readiness (no stream
  splicing), (2) frozen-fresh detection -> withhold identical frames so
  stale fallback fires. Plus the already-live laptop-side output step guard.
- ✅ DEPLOYED TO ROBOT 2026-08-12 (this session): guarded
  policy_node_29dof.py overlaid into all 4 image copies (src + both
  install spaces + build) and docker-committed; verified in-image
  (guard markers present, md5 f76c276c...4c matches laptop file at all
  runtime paths). deploy_to_robot.sh now also ships policy_node_29dof.py
  into ~/humanoid_policy_patched/ so future full deploys carry it.
- NEXT RUN PROTOCOL (the decisive instrumented run): arm laptop reference
  recorder + robot telemetry + torque log; Claude verifies stream clean
  LIVE and calls "press B now"; calm = DONE, shaking = tapes bracket the
  guilty hop. Never press B within 30 s of a STALE event.

## WHERE WE ARE
Migrating the G1 PICO-4 teleop stack from HoloMotion v1.4.0 to NVIDIA
GEAR-SONIC (GR00T-WholeBodyControl, ~/GR00T-WholeBodyControl).
Laptop side: DONE and verified. Robot side: waiting on ONE physical step —
the Orin JetPack 6.2 reflash (user hands + an external USB drive).
Detailed runbook: **MIGRATION-SONIC.md** (read it first).

## STATE RIGHT NOW
- Laptop: `.venv_teleop` (CPU torch) verified; `run_sim_loop.py` (MuJoCo)
  and `pico_manager_thread_server.py` both run; launchers at
  scripts/sonic_sim.sh + scripts/sonic_teleop.sh.
- Flash image g1-nx-j6.2.img.bz2 (3.8G) downloaded + bzip2-verified at
  ~/orin-reflash-backup/jetpack6/Jetpack6.2/.
- MISSING: Jetpack_6.2_nx.tar.bz2 (phase-2 package, ~10-12G) — laptop disk
  cannot hold it + extraction; NEEDS AN EXTERNAL USB DRIVE (or approve
  deleting the 9.1G HoloMotion fallback per MIGRATION-SONIC.md).
- All irreplaceables backed up in ~/orin-reflash-backup/ (HoloMotion
  pristine tar.gz md5-verified, robot-only fall footage, Unitree factory
  bundle, kc_ws, robot identity snapshot). Robot is SAFE TO WIPE.
- Robot: powered off; still on the old stack (JetPack 5.1 + patched
  HoloMotion image with stock C++ binary — fully working fallback).
- Old laptop stack: holosim/holosim-chain systemd units disabled;
  holosim-pcservice KEPT (SONIC uses the same XRoboToolkit PC service).

## ACHIEVED (2026-08-10 → 08-12, compressed)
- Built a full MuJoCo smoke harness for the deployed stack (bench/simsmoke,
  6/6 gating scenarios) + live headset→sim teleop with auto-tracking.
- Reached LIVE robot teleop; velocity mode good; tracking shook violently.
- Root-caused the shaking through five real stacked faults: factory 'ai'
  controller never released by L2+R2 (use release_factory_control.sh —
  gated in session_up.sh); humanoid_control C++ NOT rebuildable from
  shipped source (stock binary restored); g1-dance killswitch killing the
  app on L2+B; A-press unblended target snaps; limit_scales 2.0.
- Final root causes: (A) reference-stream teleports (1.34 m/frame,
  measured) from PICO tracking loss + node-restart splicing — pipeline
  issue that SONIC replaces; (B) hardware SUSPECTED then CLEARED (running
  test; earlier knee-torque evidence was confounded by in-place stepping).
- 40-agent audit: 27 confirmed findings (AUDIT-2026-08-11-night-report.md)
  — now largely superseded by the migration, kept for the record.
- Migration: feasibility verdict (laptop=streamer+sim, Orin=deploy host,
  TRT version must be EXACT), laptop env installed and verified, all
  backups evacuated, flash image fetched+verified, bootstrap + launcher
  scripts written.

## TODO — NEXT SESSION, IN ORDER
1. Plug in external USB drive → download Jetpack_6.2_nx.tar.bz2 to it
   (gdown, folder link in MIGRATION-SONIC.md) → extract there.
2. FLASH DAY checklist in MIGRATION-SONIC.md: SSD out → dd (VERIFY lsblk
   first!) → SSD in → recovery mode (lsusb: "NVIDIA Corp. APX") →
   flash_nx_module.sh → reassemble → nvpmodel -m 0 + jetson_clocks.
3. ssh keys to the new Orin, then run ~/orin-reflash-backup/
   orin_bootstrap.sh on it (build + checkpoints + env check, ~60-90 min).
4. Bring-up ladder (MIGRATION-SONIC.md): sim2sim → teleop-in-sim →
   real robot. PICO controls: calibration pose → A+B+X+Y engage → A+X
   POSE. E-stop: A+B+X+Y or keyboard 'O'. Needs: 2 ankle trackers
   paired+calibrated, TIGHT pants (tracker line-of-sight), room hygiene
   (50 Hz tracking lock, bright light, clean IR lenses).

## DISK CLEANUP (user asked; ~8.6G free now; candidates identified,
## NOT yet deleted — confirm before removing)
- ~/g1-dance/third_party 4.5G (mujoco_menagerie 2.3G, GMR 1.5G — git
  re-clonable) and ~/g1-dance/data/body_models 2.6G (SMPL — re-download-
  able) → ~7G if the dance project stays dormant.
- ~/Downloads: paxini_hand_sdk 422M + its .deb/.zip (~700M, re-download-
  able), chrome deb 128M. KEEP the MJ dance reference videos.
- Done already: conda pkgs + playwright caches (~3G reclaimed).

## DURABLE TRAPS (also in memory files)
- Laptop must NEVER join g1-teleop Wi-Fi. Headset→laptop = 192.168.123.2
  via robot NAT (topology note in memory).
- ONE SDK client per PC service; Mode resets in the app on every reconnect.
- pkill -f patterns that appear in your own command line self-kill the
  shell (bit us 5+ times — use ps+awk exact match or bracket trick).
- git-lfs required for GR00T repo binaries (ld "syntax error" = LFS stub).
- TRT version EXACT or silently wrong actions (SONIC docs, danger-boxed).
- NO CloudXR anywhere in our path (that's the Thor-only variant, stripped
  from the installer). All traffic is LAN: headset -> robot AP -> NAT ->
  laptop PC service. Post-reflash the AP must be restored (bootstrap step
  9 does it, incl. rtl8852bu driver rebuild fallback from the factory
  backup if wlan0 is missing on the new kernel).
