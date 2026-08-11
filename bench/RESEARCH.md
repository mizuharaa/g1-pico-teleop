# CPU retargeting for PICO → G1: research synthesis (2026-08-07)

Goal: replace HoloMotion's CUDA-locked retargeter with a CPU path — sub-ms
solve, plug-and-play — so the no-robot rehearsal runs on the GPU-less laptop.
Full agent reports: this file is the digest; details in session transcript.

## Leaderboard (measured on THIS laptop via ik_bench.py, unless noted)

| Backend | p50 | p99 | Sub-ms | Parity w/ HoloMotion | Effort |
|---|---|---|---|---|---|
| mink, 6 tasks + posture, daqp | 219 µs | 294 µs | **PASS** | approximation (needs audited target table wired in) | ~1-2 days |
| `backends:warp_cpu` (HoloMotion's own LM on Warp CPU) | 3.21 ms | 3.31 ms | fail | **exact solver**, but R_y180 quat bug in this path | works now |
| GMR (mink-based, 2-stage, ≤10 iters/stage) | est. 15–28 ms (desktop CPUs, README) | — | fail | different solver; proven PICO+G1 fidelity (TWIST2 stack) | pip + ~25 lines |

All three fit a 50 Hz budget (20 ms); only mink meets sub-ms.

## Key facts

- **HoloMotion's solver is 1 LM iteration/frame**, 125 residuals × 35 dofs,
  35×35 Cholesky, warm-started. Audit verdict: "genuinely GPU-dependent:
  essentially nothing." Full port spec (target table, weights, scales,
  rot offsets, ground sliding-min, root re-seed, 2π/slew post-processing)
  is in the audit report; target table source:
  `HoloMotion/holoretarget/assets/target_configs/smplx_to_g1.json`.
- **Dormant CPU path exists upstream**: `HoloRetargetRunner.retarget_qpos_from_pico_body_poses`
  (bypasses the CUDA-guarded `HoloPicoGpuTargetRunner`). Needs the 5-line
  pinned-memory patch in `_engine_impl.py` (gate `pinned=True` on
  `wp.is_cuda_available()`). **KNOWN BUG: applies only R_x90 to source
  quats; production GPU kernel applies R_x90 ⊗ q ⊗ R_y180 → skeleton is
  yaw-flipped 180° until fixed.**
- **PICO input**: body_poses[24,7], SMPL-24 order, xyzw quats, Unity frame.
  Position remap (x,−z,y); quat wxyz + R_x90 pre, R_y180 post.
- **Output**: qpos[36] = root pos(3) + root quat wxyz(4) + 29 joints in
  `UNITREE_G1_29DOF_NAMES` order.
- mink ships `examples/humanoid_g1.py` (whole-body G1, 200 Hz, daqp).
- GMR = mink-based, MIT, first-class PICO (`xrobot_utils.py:XRobotStreamer`,
  `ik_configs/xrobot_to_g1.json`); it is TWIST2's retargeter; ICRA'26 paper:
  91.2 mm median tracking error on LAFAN1→G1 (beats PHC/ProtoMotions).

## Ecosystem highlights (from the Isaac/MuJoCo audit)

- **GR00T-WholeBodyControl / SONIC (NVIDIA)**: same topology as ours
  (PICO + trackers → no-GPU streamer PC → policy on the G1's Orin).
  Apache-2.0, HF checkpoints (200 ms + 80 ms low-latency), JetPack 6 guide,
  and a documented **"Teleop in SIM" MuJoCo mode** = robot-free rehearsal
  path. Primary A/B candidate vs HoloMotion.
- **TWIST2**: exact hardware match (PICO 4 Ultra + calf trackers, G1 29dof),
  MIT, released ONNX policy; GMR retarget → 35-D mimic obs → Redis → policy.
  Their empirical tip: pass operator height ~lower than real (PICO
  underestimates; they use 1.6 m).
- **HoloMotion official finetune recipe**: `holomotion/scripts/training/
  finetune_motion_tracking_v1_4_0.sh` resumes HF checkpoint `model_14000.pt`,
  10k iters, Isaac Sim 5.0 headless. Cloud cost: one 4090/A100-80GB,
  under a day, ~$10–50 spot. This is the shoulder/hip-fidelity fix path
  (add reward points for shoulder/hip links, retrain).
- **WebXR cannot read PICO Motion Trackers** — browser paths (xr_teleoperate,
  Vuer, TeleVision) are upper-body only, forever. Full body ⇒ native
  XRoboToolkit route.
- xr_teleoperate v1.6: still arms+hands+head only. Complement, not rival.
- Skip: HOVER (H1, dormant), OpenWBT (Vision Pro), OpenHomie (exoskeleton),
  ExBody2 (no deploy code). Watch: IsaacTeleop (NVIDIA+PICO device layer),
  unitree_rl_lab, AMO/GMT (sim-only references).
- In-headset MuJoCo (Unity plugin + PICO OpenXR): pioneering territory,
  weeks of work, nobody has done PICO+body-tracking+humanoid. Use SONIC's
  SIM mode instead.

## DONE 2026-08-07 — CPU rehearsal path is LIVE

Requirement relaxed: sub-ms dropped; must fit a 50 Hz loop (20 ms). Done:

1. **R_y180 parity fix applied** to all 5 NumPy conversion sites in
   `holoretarget/_engine_impl.py` (marker: `# LOCAL PATCH R_y180`).
   Verified EXACT vs a transcription of the GPU kernel: worst quat error
   2.2e-16 over 50 random trials. (6th site = env-gated native C path,
   default-off, NOT fixed — do not set HOLORETARGET_NATIVE_DIRECT_TARGETS.)
2. **CPU fallback wired into `holoretarget/online.py`** (try/except around
   the CUDA-only HoloPicoGpuTargetRunner; `.orig` backups beside both
   patched files). GPU behavior unchanged.
3. **End-to-end**: production `holomotion_teleop_node.py --fake-pico-stream`
   runs at a steady 50 Hz on the laptop, tick_total ≈ 5.0 ms
   (solve 3.5 ms), ZMQ reference_qpos publishing on 6001.

⇒ `scripts/start_rehearsal.sh` now works on this laptop. Only trackers
are missing for the real rehearsal.

Note: mink port (294 µs) no longer needed for the rehearsal; keep as a
latency reserve. GMR remains the fidelity reference.

## Shoulder/hip fidelity: order of operations

Cloud finetune does NOT need tracker calibration first — training consumes
retargeted mocap datasets, not live PICO data. But run the diagnostics
first so the $10–50 buys the right fix:
1. (no trackers needed) sim2sim per-joint tracking error of the pretrained
   policy on reference motions — quantifies the POLICY-side shoulder/hip gap.
2. (trackers) rehearsal + mjviewer shows the retargeted REFERENCE, pre-policy.
   Reference bad → fix sensing/target-table weights (free). Reference good,
   robot bad → policy is the culprit → cloud finetune with added
   shoulder/hip reward key-bodies.
