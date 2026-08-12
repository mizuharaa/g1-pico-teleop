# G1 Teleop Stack — Final Audit Report (2026-08-11)

**Context:** 20 confirmed findings (8 critical-class), 7 refuted, ~20 minor. The 8/11 "policy never tracked reference" scare is resolved: the command path /humanoid/action→motors is VERIFIED WORKING (PD hold against manual displacement, tau opposing, exact return-to-target); the actual break was that the policy never entered motion mode. The waist_pitch motor thermally tripped TWICE on 8/10 (130 C, error 512, torque reapplied seconds after each recovery); flag is now clear but gears uninspected. The supervisor's reconnect/bounce logic has never executed (ss column bug), and its kill path leaks the single SDK client slot.

---

## (1) Ranked actions before next hardware attempt

**BLOCKERS — code fixes (highest risk-reduction per line):**

1. **A-button snap (policy_runtime.py:164-165, 197-212).** Guard the A edge with `not policy_enabled` (or route to `switch_to_velocity_mode` when mode=='motion'); add `mode_blend_from_real` capture AND clear stale `mode_blend_t0` in `enable_velocity_policy`. This single fix kills the remaining unblended-lunge path AND the log-verified 7 mid-run re-enable snaps in 4.4 s during today's shaking (8x "Policy enabled" burst, container 0811_165852). Also note `reset_counters` clears the slew anchor, so slew=15 does not soften this snap.
2. **NaN chain.** (a) `_publish_action_target` (policy_node_29dof.py:1370-1381): early-return + hold-last-good + trigger damped e-stop on NaN — currently logs and publishes anyway. (b) isfinite gate in main_node.cpp `PolicyActionHandler` (~729: NaN skips std::clamp, lands in motor q at full kp). (c) Fix `_slew_prev_target` NaN poisoning (one NaN frame is sticky). (d) remote_controller_filter.py: add positive vx clamp (only `vx < -0.5` exists), isfinite checks on the four unpacked floats, and NaN-proof the EMA (NaN prev is permanent).
3. **Frozen-fresh stream detection.** Verified: 1.6 s bit-identical reference freeze while pico_dt self-reported ~14 ms; age-based staleness (`get_with_age_and_delay`) is blind to it, and the post-stall frame carried 43 deg quat + 327 mm root in one consumed tick. Add content/frame_index-progress staleness in `ZmqReferenceBuffer` + a per-tick output step clamp (root/quat/joint) or ramp gate on resume; add frame_index-regression detection in `vr_reference.store()` (sub-0.6 s publisher restarts currently blend pre/post-restart poses — 1.34 m root steps recorded).
4. **Supervisor script (run_rehearsal_supervised.sh) — five one-liners, do together:**
   - `awk '{print $4}'` at lines 85 and 93 (ss omits State under a state filter; bounce has never fired)
   - line 101: `graceful_kill` not `kill -9` (fixing the awk bug alone makes every reconnect leak the SDK slot)
   - add `200>&-` to lines 59 and 160 (flock leak → permanent supervisor lockout via orphaned viewer/service)
   - line 119: also re-arm SEEN_DATA on head_pose_ok-without-body-data (otherwise 12 s kill loop while operator needs 60-90 s to re-tick Mode=Full-body)
   - holomotion_teleop_node.py init_xrt loop (1014-1041): check stop_event, xrt.close() on early exit (SIGTERM currently ignored → every init-phase restart is a slot-leaking SIGKILL; this is what makes the kill loop unwinnable)
5. **Limits.** g1_29dof_holomotion.yaml limit_scales 2.0→1.0 (comment says "50% more", code doubles the half-range); add torque clamping to POLICY state in main_node.cpp (currently none; velocity_limit_scale is loaded and never applied anywhere).
6. **ZMQ subscriber immortality.** reference_transport.py:312-343: per-packet try/except inside the recv loop (or is_alive watchdog + respawn) — one malformed packet permanently kills reference input with one log line.

**BLOCKERS — telemetry (next incident is currently unforensicable):**

7. Run with `launch_runtime.sh --record` AND volume-mount/docker-cp `bag_record` out of the container (lowstate mcap contains raw wireless_remote — resolves mash-vs-flicker and any future sprint). Add throttled log of raw lx/ly/rx/ry + computed vx/vy/vyaw.
8. joint_watchdog.py: log_every_s 0.2→0.01 for tau (5 Hz CSV Nyquist=2.5 Hz made today's 2.1-2.5 Hz shake peaks unattributable); populate staleness_s (line 175 hard-codes "").
9. Log effective env knobs (SLEW/SOFT_START/BLEND, HOLOTELEOP_REF_SMOOTH) at node startup.

**BLOCKERS — hardware/procedural:**

10. **Inspect waist gearing** (two 130 C thermal trips within 2 h on 8/10, ~29 Nm reapplied seconds after each recovery); check ankles cool (left_ankle_pitch hit 89 C — above the watchdog's own 85 C ALARM). Verify motorstate flags 0 at power-on (log says clear).
11. **Verify mode switching, not the command path**, as the first on-stand test: B-press into motion mode with robot suspended, confirm `[Tracking] mode=motion` in robot-side logs and reference-correlated targets before any free-standing attempt.

**SHOULD-FIX (same session if possible):**

12. Button hygiene: header 0x55 0x51 check + 2-frame debounce in both wireless parsers (single corrupt frame can press A/B/Start; C++ side is level-triggered so partially protected by state guards).
13. Wrist IK flip-flop: 87/85 reversal events, 55-frame 25 Hz bang-bang runs at 0.2-0.7 rad on left wrist — add hysteresis/rate-limit in the retargeter or clamp wrist dofs before arm tracking is restored.
14. Null `target_dof_pos_real` + `mode_blend_*` in EMERGENCY_STOP (post-e-stop quick A→B snaps to stale pre-e-stop pose).
15. Clip-injection fallback: the 6 ref_* obs getters' third branch must check `reference_stream_active` and fail safe (zeros/velocity-switch) instead of serving offline-clip data on transient kinematics=None.
16. Env parsing: reject nan/inf in HOLOMOTION_* floats; note start_teleop_container.sh hardcodes slew=15 and silently ignores host exports of all three knobs.
17. start_rehearsal.sh: add the flock guard + --skip-start-service (live re-trigger of the multi-node SDK-slot fight) or delete it.
18. Add applied-vs-raw action discrepancy telemetry (last_action obs feeds RAW actions while soft-start/blend/slew shape the published target — deliberate but unbounded and currently unobservable; suite passes but never enters tracking with a large operator-pose offset).

## (2) Go/No-Go

**NO-GO** for free-standing teleop as the stack sits.

**Conditional GO** for an on-stand (suspended/gantry) session once: items 1-6 merged and re-validated by the sim suite + new tests (A-during-motion, NaN frame, frozen-fresh stream); items 7-9 recording; item 10 inspection done. On-stand session scope: verify motion-mode entry (item 11), soft-start behavior with a real operator-pose offset, and frozen-stream fallback by killing the publisher mid-track.

**GO for free-standing** only after the on-stand session shows: zero unblended target steps >0.15 rad at every mode transition, frozen-stream fallback firing on content-staleness (not just age), waist/ankle temps <70 C throughout, and a working software killswitch drill (factory L2+B is dead under custom control — per standing memory).

## (3) Sim harness additions (bench/simsmoke)

1. **Gap 1 — bus contention:** model lowcmd as N-publisher stream with firmware interleaving; add a factory-controller rig publishing 500 Hz damped-stand PD alongside the FSM (reproduces the shaking class); MotionSwitcher stub — fake firmware rejects external lowcmd until released; gating scenario FAILS unless CheckMode empty before Start.
2. **Gap 2 — SIL:** run the deployed humanoid_control binary (container build from same source+flags; qemu if feasible) against a CycloneDDS↔G1MujocoRobot bridge; cheaper: differential gate feeding identical lowstate/gains/action streams to binary and MainNodeSim, FAIL on per-tick lowcmd divergence >epsilon.
3. **Gap 3 — CRC:** FSM emits struct-packed LowCmd with real motor_crc_hg CRC; fake firmware deserializes, verifies with an independent implementation, drops bad frames and holds last-valid; gate `rejected==0 and accepted>0` in every scenario (bad rebuild then shows as "never leaves ZERO_TORQUE" — today's real signature).
4. **Gap 5 — plant realism + gating:** one-tick command and state latency; encoder quantization + Gaussian noise on q/dq (kd chatter substrate); validate/tune existing armature/frictionloss against a logged real stand, add backlash; **promote max_target_step and a quiet-stance tau-ripple std to GATING thresholds in every tracking scenario** — instrumented runs already show 0.33 rad/20 ms in quiet stance, so this gate fails today and forces the smoothing question before hardware.
5. **Gap 6 — timing:** latency model on policy step (target delivered N~dist ticks late, N in 1..10), random dropped cycles, jitter sweep during tracking with Gap 5 gates active.
6. **Gap 7 — remote decode:** route the already-built byte-faithful 40-byte wireless_remote payload through the FSM's parser (port the byte parse into fsm_sim until SIL exists) so e-stop chord decoding is actually tested.
7. **New scenarios mapping to confirmed findings:** A-press mid-motion-mode (the snap); A-mash burst (8 edges in 6 s); single-NaN action frame (assert hold + e-stop, no publish); frozen-fresh reference (bit-identical frames with fresh timestamps for 2 s → assert fallback); publisher restart with frame_index regression at dt<0.6 s; post-e-stop quick A→B; B-press with large operator-pose offset (soft-start worst case, currently untested); corrupted wireless_remote frame with a button bit set.
8. **Test fixes:** tighten test_slew_limiter from `slew*4*dt` to ~`slew*dt`; add blend+slew combined test; run tests against the container-effective env (slew=15) not code defaults.

**Key files:** /home/alois/full-body-teleoperation/HoloMotion/deployment/unitree_g1_ros2_29dof/src/humanoid_policy/{policy_runtime.py, policy_node_29dof.py, observation_evaluator.py, reference_transport.py, utils/remote_controller_filter.py}, .../src/src/main_node.cpp, .../src/config/g1_29dof_holomotion.yaml, /home/alois/full-body-teleoperation/scripts/{run_rehearsal_supervised.sh, start_teleop_container.sh, joint_watchdog.py, start_rehearsal.sh}, /home/alois/full-body-teleoperation/HoloMotion/deployment/holomotion_teleop/holomotion_teleop_node.py, /home/alois/full-body-teleoperation/bench/simsmoke/{run_smoke.py, g1_mujoco.py, fsm_sim.py}. Evidence data: /tmp/session_monitor/watchdog_151420.csv, /tmp/ref_rec.bin, /tmp/targets_rec.csv (note: recorded 15:44-15:46, NOT the shake window), ~/Videos/g1-demos/20260811_155051_*.mp4 (covers the pre-16:16 forensic blind spot).