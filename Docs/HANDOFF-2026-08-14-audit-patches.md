# G1 PICO Teleop — Independent Audit, Patch Set, Direction

**Date: 2026-08-14. Author: independent review pass (Claude Opus 5).**
**Scope: read-only audit of the repo at `b08b485` + 5 patches. No robot action taken, nothing deployed.**

Companion to `HANDOFF.md` (2026-08-12 night). Where this document disagrees
with the prior record, the disagreement is stated explicitly in §1 rather than
quietly overwritten.

---

## 0. TL;DR

Five patches, tiered by whether they can actually reach the robot today:

| # | File | Fixes | Deployable now? |
|---|---|---|---|
| P1 | `src/config/g1_29dof_holomotion.yaml` | `limit_scales` 2.0 → 1.0 | **Yes** — read at runtime by the stock binary |
| P2 | `src/humanoid_policy/policy_runtime.py` | A-button re-entry snap | **Yes** — python overlay path |
| P3 | `src/src/main_node.cpp` | dead A-press safety gate | No — needs C++ rebuild |
| P4 | `src/src/main_node.cpp` | no torque limiting in POLICY | No — needs C++ rebuild |
| P5 | `src/src/common/motor_crc_hg.cpp` | **the rebuild blocker itself** | Unblocks P3/P4 |

P1 alone is expected to remove the "tippy-toe lean at A" symptom, and it costs
one config edit and one container restart. **Do it before the JetPack 6.2
reflash**, because its result is diagnostic (see §5).

---

## 1. Correction to the record: the exoneration does not cover the symptom

The repo currently holds three mutually exclusive verdicts:

- `HANDOFF-archive-2026-08-12.md` (evening): *"SOFTWARE EXONERATED FOR THE
  LEANING/TWITCHY STAND; VERDICT = PHYSICAL (calibration or gears)."*
- `MIGRATION-SONIC.md`: *"GATE 1 — RETIRED … hardware cleared by running test."*
- `HANDOFF.md` (current SSOT): *"The robot hardware is confirmed functional …
  All failures originate in the reference/retargeting middleware."*

They cannot all be right, and the tie-breaker matters because it decides
whether a day of Orin reflashing is the right next move.

**The load-bearing evidence for hardware exoneration is the bundled offline
clip running flawlessly twice daily. That evidence does not cover the A-stand.**

The offline clip exercises the **motion-tracking** policy against a known-good
canned reference. The "leans forward on tippy toes at A" symptom occurs in the
**velocity** policy — a different ONNX model, with a different default pose
(`velocity_default_angles_onnx` vs `motion_default_angles_onnx`, loaded from
separate metadata at `policy_node_29dof.py:1210,1218`), a different observation
set, and a different balance feedback path. A flawless motion-tracking clip
says nothing about velocity-policy behaviour at mode entry.

Symmetrically, the 08-12 "verdict = physical" argument is a *differential*: no
software changed between the flawless 08-10 run and the bad 08-12 stand, so the
delta must be physical. That argument is sound as far as it goes, and this
audit does **not** refute it. `limit_scales: 2.0` was also 2.0 on 08-10, so it
cannot by itself explain what *changed*.

**The reconciliation both documents miss:** `limit_scales: 2.0` plus the absent
POLICY torque limiter is an **amplifier**, not a trigger. It does not create the
error; it converts a recoverable one into a joint driven 2.33× past its hard
stop at kp≈300 with no torque cap. So:

- A physical cause (calibration drift after the face-fall) remains live.
- The amplifier is what turns it into tippy-toes, 130 °C, and fault 512.
- Fixing the amplifier is worth doing **either way**, and it is the cheapest
  available experiment that discriminates between the two verdicts (§5).

---

## 2. Confirmed findings (verified in source, with arithmetic)

### 2.1 `limit_scales: 2.0` expands about the midpoint — exactly explains +1.222 rad

`main_node.cpp` `PolicyActionHandler` (lines 719-727) does not clamp to the
joint limits. It expands the range about its **midpoint**:

```cpp
double mid_pos          = (lo + hi) / 2.0;
double half_range       = (hi - lo) / 2.0;
double scaled_half_range = half_range * position_limit_scale;   // × 2.0
double max_pos          = mid_pos + scaled_half_range;
```

For `left_ankle_pitch_joint: [-0.87267, 0.5236]`:

```
mid              = -0.174535
half_range       =  0.698135
scaled (×2.0)    =  1.396270
max_pos          = -0.174535 + 1.396270 = +1.2217 rad
```

**+1.2217 rad is precisely the "+1.222 rad commanded vs +0.524 physical max"
measured on the A-stand** (`AUDIT-2026-08-11.md`). The ankle was not merely
over-commanded — it was pinned at the scaled clamp ceiling, which means the
upstream command was ≥ the ceiling and the clamp was the only thing bounding it.
Positive ankle_pitch is plantarflexion: heels lift. That is the tippy-toe.

Same arithmetic elsewhere:

| Joint | Physical range | Range permitted at 2.0 |
|---|---|---|
| `left_ankle_pitch_joint` | −0.873 … +0.524 | −1.571 … **+1.222** |
| `left_knee_joint` | −0.087 … +2.880 | −1.571 … **+4.363** (250°) |
| `waist_pitch_joint` | ±0.52 (±30°) | **±1.04 (±60°)** |

`waist_pitch` is the joint that reached 130 °C with fault 512, twice. At kp=200,
a target 0.52 rad beyond its stop demands ~104 Nm from a 35 Nm actuator.

The stock comment claims *"Allows 50% more range of motion."* The code doubles
the half-range. The comment is wrong about its own units.

### 2.2 The A-press safety gate is dead code

`main_node.cpp:418-421` gates the MOVE_TO_DEFAULT → POLICY transition on lower-body
joints being within 0.4 rad of default. The list reads:

```cpp
"left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", ...
```

but `complete_dof_order[i]` yields `left_hip_yaw_joint`, `left_knee_joint`, …
(see `g1_29dof_holomotion.yaml`). `std::find` never matches, every joint hits
the `continue`, the loop inspects nothing, and `positions_ok` is unconditionally
`true`. **A always enters POLICY**, regardless of leg pose. Fixed in P3.

### 2.3 POLICY state applies no torque limiting at all

`main_node.cpp:630` — stock comment: *"Use policy kp/kd values directly without
torque limiting."* `limitTorque()` / `limitTorqueWithCustomGains()` are
implemented and used in MOVE_TO_DEFAULT, but **never called in POLICY**, which
is the state the robot actually runs in. A joint railed against its stop sits at
a large persistent error against kp=300 with no cap → sustained saturation
(heat) and, with kd=5 against kp=300, an underdamped limit cycle. The audit's
measured 2.1-2.5 Hz oscillation is consistent with this. Fixed in P4.

### 2.4 NaN reaches the motors

`PolicyActionHandler` has no finiteness check, and `std::clamp` does not
sanitise NaN — `NaN < min` and `NaN > max` are both false, so the guard is
skipped and NaN lands in `target_dof_pos`, from where `SendPolicyCommand`
publishes it at full kp. No `isfinite` guard exists anywhere in
`policy_node_29dof.py` either. Fixed in P3's file (`main_node.cpp`).

### 2.5 A had no re-entry guard

`policy_runtime.py:164` (stock):

```python
if self._is_button_pressed(KeyMap.A) and self.state.robot_state_ready:
    self.enable_velocity_policy()
```

No `not policy_enabled` check. Every A press — including mid-run in motion mode —
re-runs `enable_velocity_policy()`, which snaps `target_dof_pos_onnx` to the
velocity defaults with **no blend**, unlike `switch_to_velocity_mode()` which
captures `mode_blend_from_real`. Logged: 7 mid-run snaps in 4.4 s (container
`0811_165852`); trap #4 in `HANDOFF-archive-2026-08-11.md` records 8 in 5.8 s
"during shaking". Fixed in P2, with a regression test.

### 2.6 Three different "default poses" are in play

| Pose | Source | Applied at |
|---|---|---|
| 1 | `default_joint_angles` in YAML (`waist_pitch: 0.2`) | Start → MOVE_TO_DEFAULT |
| 2 | `velocity_default_angles_onnx` (velocity model meta) | A |
| 3 | `motion_default_angles_onnx` (motion model meta) | B |

A steps 1→2 **unblended**. B steps 2→3, blended only by the local 08-11 patch,
whose own comment records the residual: *"waist_pitch differs +0.107 rad from
the velocity default — a forward-lean snap that lunges the robot at every
B-press."* This is a structural discontinuity at every mode change, independent
of VR entirely.

---

## 3. The rebuild blocker, and its fix (P5)

`HANDOFF-archive-2026-08-11.md` trap #2: *"humanoid_control CANNOT be rebuilt
from shipped source (3 rebuilds, 3 failure modes: -O0 slow loop; -O2 CRC-dead
via strict aliasing; -O2+ no-strict-aliasing misbehaved)."* Consequence: the
image runs the stock Jul-16 binary (5,605,520 B) and **every C++ fix is
undeployable**. The Aug-11 rebuild (1,147,496 B) is known-bad.

The prior record identified the symptom ("CRC-dead via strict aliasing") but
treated it as a compiler-flag problem. It is a source defect, and it is one line
— `motor_crc_hg.cpp:24`:

```cpp
raw.crc = crc32_core((uint32_t *)&raw, (sizeof(LowCmd) >> 2) - 1);
```

Casting `LowCmd*` to `uint32_t*` and dereferencing violates strict aliasing. At
-O2 the compiler may assume the `raw.motorCmd[i]` stores cannot alias the
`ptr[i]` loads inside `crc32_core`, and reorder or elide them — CRC computed
over a partially-written struct → firmware rejects every `LowCmd` → **the robot
never leaves ZERO_TORQUE**, which is the recorded bad-rebuild signature.

`-fno-strict-aliasing` suppresses the symptom without fixing the UB, which is
consistent with the third rebuild "misbehaving". P5 replaces the cast with a
`memcpy` into a real `uint32_t` array — well-defined at every optimisation
level, same bytes, same length, same trailing word excluded:

```cpp
constexpr size_t kWordCount = sizeof(LowCmd) / sizeof(uint32_t);
std::array<uint32_t, kWordCount> words{};
std::memcpy(words.data(), &raw, sizeof(LowCmd));
raw.crc = crc32_core(words.data(), kWordCount - 1);
```

**If P5 holds, the C++ patch path reopens** and P3/P4 become deployable via the
existing `scripts/deploy_cpp_patch.sh`.

`bench/crc_aliasing_check.cpp` is a standalone gate for this — build at -O0 and
-O2 and compare. **This was not compiled during this audit** (no toolchain on
the review machine); see §7.

---

## 4. Coding suggestions beyond the patches

1. **Delete `limit_scales` as a concept, or rename it.** A "limit scale" that
   expands about the midpoint silently moves the *lower* bound too:
   `waist_pitch` at 2.0 permits −60° as readily as +60°. If headroom is ever
   genuinely needed, scale toward one bound explicitly and document the sign.

2. **Make mode entry a ramp, not an assignment.** All three default poses should
   be reached by the same blended path. Right now MOVE_TO_DEFAULT ramps (it has
   `ratio = clamp(time_/duration_, 0, 1)`) but A and B assign. One shared
   `blend_to(target, duration)` used by all three transitions would delete this
   entire class of bug rather than patching each entry point.

3. **The C++ node has no action-staleness watchdog.** `target_dof_pos` is only
   written by `PolicyActionHandler`. If the policy node dies, `SendPolicyCommand`
   keeps publishing the last target at full kp **forever**. Add a
   last-action timestamp and fall to damping after ~200 ms. This is a bigger
   safety hole than anything in the VR chain, and it is entirely robot-local.

4. **`for (const auto &pair : target_dof_pos)`** iterates a `std::map` — string
   comparison per joint per tick at 500 Hz, and iteration order is alphabetical
   rather than motor order. It is correct (it indexes via `dof2motor_idx`) but
   it is doing avoidable work in the hot loop; a flat `std::array<double, 29>`
   indexed by motor id would be both faster and harder to get wrong.

5. **Check gains for damping ratio, not just magnitude.** kp=300 with kd=5 on
   the knee/ankle is far from critically damped. If P1+P4 reduce but do not
   eliminate the 2-3 Hz chatter, kd is the next knob — raising leg kd toward
   15-25 is more likely to help than lowering kp.

---

## 5. Direction

**Recommendation: run the 30-minute config experiment before the reflash, then
migrate to SONIC regardless — but do not expect SONIC to fix the session/transport
failures.**

### 5.1 SONIC inherits the entire transport failure class

`HANDOFF.md` §7 open question 6 asks whether SONIC's reference path shows the
same transition-window failures. **It does.** SONIC uses the same
XRoboToolkit PC service, the same PICO app, and the same ZMQ transport
([VR teleop setup](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/vr_teleop_setup.html),
[ZMQ streaming](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/zmq.html)),
and `MIGRATION-SONIC.md` already commits to keeping the existing
`holosim-pcservice`. Therefore every failure in `HANDOFF.md` §3.3-3.4 —
one-client SDK slot leak, `BodyDataAvailable` write-once latch, headset
proximity sleep, corporate Wi-Fi auto-hop, app IP field reversion — lives in
XRoboToolkit, **not** in HoloMotion, and carries over unchanged.

The guards in `HANDOFF.md` §6 (frozen-fresh detection, frame_index regression
reset, SIGTERM-first bounces) are therefore **not throwaway work** — they are
the parts most worth porting, because they address the layer that is not being
replaced.

### 5.2 What SONIC should genuinely improve

- **The executor.** New command path, no `limit_scales: 2.0`, no missing torque
  limiter. §2.1-2.4 stop being your problem.
- **Balance on hard poses.** The kungfu / one-leg-stand / kick stepping-forward
  behaviour is a known G1 whole-body-control limitation — the locomotion policy
  is unaware of arm state and shuffles forward trying to rebalance
  ([Robotics Knowledgebase](https://roboticsknowledgebase.com/wiki/common-platforms/unitree-g1/)).
  A behaviour foundation model trained on heterogeneous whole-body data is a
  reasonable bet here, and this is the one complaint no amount of HoloMotion
  patching addresses.

### 5.3 Why to run P1 first anyway

The reflash is a day and a disk wipe. The P1 experiment is one YAML edit and a
container restart, and its outcome **discriminates between the two live verdicts**:

- If tippy-toe **disappears** at `limit_scales: 1.0` → the A-stand was command
  authority. The 08-12 "physical" verdict was over-called, calibration is fine,
  and you carry a known-good config into SONIC.
- If tippy-toe **persists** → command authority was never the trigger, the
  08-12 differential argument stands, and you have strong evidence for
  calibration/gear inspection **before** committing the Orin to a reflash you
  would otherwise be debugging on top of a physically-off robot.

Either answer is worth 30 minutes. Neither is obtainable after the wipe.

---

## 6. Test plan

Gantry on, spotter present, remote in hand, `estop_console2.sh` armed. No VR
in the loop for steps 1-3 — these are robot-local faults and VR only confounds.

1. **P1 only.** Edit `limit_scales` → 1.0, restart container. Start → wait for
   MOVE_TO_DEFAULT to settle → **A once**. Watch heels.
   - Record: ankle_pitch commanded vs measured, waist_pitch temperature.
   - Expected: ankle command bounded at +0.5236 instead of +1.2217.
2. **P2.** Deploy python overlay (remember the extensionless entry copy —
   `HANDOFF.md` §3.6). Press A three times in POLICY; confirm no re-snap in the
   target trace and no log burst.
3. **B-press, still no VR.** Confirm whether twitching persists with a
   stale/absent reference. This isolates §2.6 (pose discontinuity) from the VR
   chain — the prior record never ran this separation.
4. **P5 gate.** Build `bench/crc_aliasing_check.cpp` at -O0 and -O2. If MATCH at
   both, rebuild `humanoid_control` via `deploy_cpp_patch.sh`. Verify the robot
   leaves ZERO_TORQUE on Start **before** any policy runs.
5. **P3/P4.** Only after step 4 passes. Re-run steps 1-3.
6. **Only then** reintroduce VR.

---

## 7. What this audit did NOT verify

Stated plainly so the next reader does not inherit false confidence:

- **Nothing was run on hardware.** No robot action, no deployment, no container.
- **The C++ patches (P3/P4/P5) were never compiled.** No toolchain was available
  on the review machine (no g++, no clang, no WSL distro). They are reviewed
  code, not built code. Step 4 above is the real gate.
- **`bench/crc_aliasing_check.cpp` has never been executed.** Its stand-in struct
  demonstrates the hazard pattern; it is not the real `LowCmd`.
- **P1/P2 are verified only against the existing unit tests** — 16/16 in
  `tests/test_policy_runtime.py` pass, including a new regression test. Seven
  suites could not run at all on the review machine (missing `torch`,
  `omegaconf`, `yaml`, `pytest`); those failures are environmental and predate
  these patches.
- **The physical hypothesis is untested and remains open.** Joint calibration
  and waist/ankle gear inspection after the 130 °C / fault-512 events are still
  owed, and `AUDIT-2026-08-11.md` item 10 has never been closed. This audit
  narrows *why the consequences were severe*; it does not establish that the
  robot is mechanically sound.

---

**End of Document**
