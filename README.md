# G1 PICO Teleop — full-body teleoperation of a Unitree G1 with a PICO 4

Teleoperate a **Unitree G1 EDU Ultimate (29 DoF)** humanoid with a **PICO 4
headset + 2 PICO Motion Trackers** (ankles): the robot imitates the operator's
whole body — arms, waist, squats, steps — while balancing itself.

This repository is the laptop-side workspace and the project's single source
of truth: deployment/safety scripts, the vendored HoloMotion stack with local
patches, sim benches, audits, and the full documentary record of every failure
mode found along the way.

> **New here? Read these two files first:**
> 1. [`Docs/PROJECT_STATE.md`](Docs/PROJECT_STATE.md) — what is true *today*
>    (which stack is active, what is parked, what is pending).
> 2. [`Docs/DEVELOPER-HANDOFF.md`](Docs/DEVELOPER-HANDOFF.md) — onboarding for
>    a new developer: hardware, networks, safety rules, known traps.

## ⚠️ Safety, before anything else

- **Gantry/tether + a human spotter holding the Unitree remote are mandatory**
  for every powered session. There is no reliable software e-stop.
- **The remote is the safety path, not the SDK.** `LocoClient.Damp()` from the
  laptop has failed at crunch time (unconfirmed sends) while a bridge was
  commanding the robot. The proven kill is `scripts/teleopit_estop.sh`: kill
  the command stream → firmware damps in ~2 s.
- Remote chords differ per stack — see the button maps in
  [`Docs/DEVELOPER-HANDOFF.md`](Docs/DEVELOPER-HANDOFF.md). Under the
  **Teleopit** state machine the e-stop is **L1+R1**. Under **HoloMotion**
  custom control the factory damp chord **L2+B is dead**; use **Select**.

## The three stacks

| Stack | Status | Where |
|---|---|---|
| **Teleopit** (BotRunner64) | **ACTIVE** — sim2sim + real dry-run passed 08-14; waist unlocked 08-17 | `~/Teleopit` on the laptop; local commits exported in [`teleopit-export/`](teleopit-export/) |
| HoloMotion v1.4.0 | Parked, fully functional and fully documented | [`HoloMotion/`](HoloMotion/) (vendored, with local patches), Docker image on the Orin |
| SONIC / GR00T-WholeBodyControl | Parked mid-migration (blocked on JetPack 6.2 reflash) | `~/GR00T-WholeBodyControl` on the laptop; [`Docs/MIGRATION-SONIC.md`](Docs/MIGRATION-SONIC.md) |

## Repository layout

```
Docs/               All project documentation (see index below)
HoloMotion/         Vendored HoloMotion v1.4.0 + local patches (upstream .git
                    renamed aside to .git-upstream, see .gitignore)
teleopit-export/    The 11 Teleopit local-fix commits as patches + git bundle
scripts/            Deployment, session, safety, and diagnostic scripts
bench/              Laptop-side benches: IK benchmark, CRC strict-aliasing
                    check, sim smoke tests (bench/RESEARCH.md)
tools/              crane, adb platform-tools
artifacts/          Installers: XRoboToolkit PC service .deb, PICO APKs
```

### Key scripts

| Script | Purpose |
|---|---|
| `scripts/teleopit_estop.sh` | **Proven emergency stop** for Teleopit sessions (kills the LowCmd stream; SDK damp attempted last, best-effort) |
| `scripts/g1_estop.py` | SDK damp — treats a no-reply as **unconfirmed**, tells the operator to use the remote |
| `scripts/deploy_to_robot.sh` / `scripts/robot_install.sh` | Ship the HoloMotion patch overlay to the Orin (incl. the limit_scales 1.0 config, both copies) |
| `scripts/deploy_cpp_patch.sh` | HoloMotion C++ rebuild path (ships the CRC strict-aliasing fix that unblocks rebuilds) |
| `scripts/joint_watchdog.py` / `arm_watchdog.sh` | Joint telemetry watchdog (alert-only: auto-kill of an untethered robot = collapse) |
| `scripts/waist_pitch_probe.py` | Two-direction low-torque probe of waist_pitch (the tool that led to finding the waist LOCK) |
| `scripts/session_up.sh`, `monitor_session.sh`, `record_session.sh`, `session_snap.py` | Session bring-up, monitoring, recording |
| `scripts/sonic_sim.sh` / `sonic_teleop.sh` | SONIC-track launchers (CPU laptop constraints baked in) |

## Documentation index (`Docs/`)

**Current:**
- [`PROJECT_STATE.md`](Docs/PROJECT_STATE.md) — state of the project, 2026-08-18
- [`DEVELOPER-HANDOFF.md`](Docs/DEVELOPER-HANDOFF.md) — onboarding handoff

**HoloMotion track (parked; operational docs kept intact):**
- [`RUNBOOK.md`](Docs/RUNBOOK.md) — full HoloMotion session runbook
- [`INSTRUCTIONS.md`](Docs/INSTRUCTIONS.md) — simplified operator instructions
- [`SETUP_STATUS.md`](Docs/SETUP_STATUS.md) — one-time installs
- [`HANDOFF.md`](Docs/HANDOFF.md) — 2026-08-12 research brief: the
  retargeting-middleware failure catalog (the deepest technical document here)
- [`HANDOFF-2026-08-14-audit-patches.md`](Docs/HANDOFF-2026-08-14-audit-patches.md)
  — independent audit + P1–P5 patch set (limit_scales, CRC rebuild blocker)
- [`AUDIT-2026-08-11.md`](Docs/AUDIT-2026-08-11.md),
  [`AUDIT-2026-08-11-night-report.md`](Docs/AUDIT-2026-08-11-night-report.md)
  (+ [full JSON](Docs/AUDIT-2026-08-11-night-full.json)) — earlier audits
- [`HANDOFF-archive-2026-08-11.md`](Docs/HANDOFF-archive-2026-08-11.md),
  [`HANDOFF-archive-2026-08-12.md`](Docs/HANDOFF-archive-2026-08-12.md) —
  archived daily handoffs

**SONIC track:**
- [`MIGRATION-SONIC.md`](Docs/MIGRATION-SONIC.md) — migration plan and status

Historical documents are kept verbatim as dated records — where a later
finding overturns an earlier one (e.g. the waist "jam" that turned out to be
the factory waist **lock**), `PROJECT_STATE.md` is authoritative.
