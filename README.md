# G1 PICO Teleop — full-body teleoperation of a Unitree G1 with a PICO 4

Drive a **Unitree G1 EDU (29 DoF)** humanoid with your own body: a **PICO 4
headset + two PICO Motion Trackers** on the ankles stream your skeleton to a
laptop, which retargets it to the robot and runs a whole-body tracking policy
at 50 Hz. Arms, waist, squats, steps and walking are all mirrored while the
robot balances itself.

This repository is the stack that actually runs the robot: a fork of
[BotRunner64/Teleopit](https://github.com/BotRunner64/Teleopit) (base commit
`f926386`, v0.5.0) plus our lab commits on top (waist unlock, stability tuning,
stick-driven walking mode, session launcher, robot-side networking). The
upstream README is kept as [`README.upstream.md`](README.upstream.md) and the
upstream docs (BVH playback, training, LinkerHand, OpenNeck, recording) live in
[`docs/docs/`](docs/docs/).

> Previous history: the project first ran on HoloMotion and briefly prepared a
> SONIC migration. That whole tree, with its audits and runbooks, is preserved
> on branch **`archive/holomotion-2026-08-18`** of this repo. The two
> hand-off documents that still matter are copied under
> [`lab/history/`](lab/history/).

---

## Contents

1. [Hardware and network](#1-hardware-and-network)
2. [Safety rules](#2-safety-rules)
3. [One-time setup](#3-one-time-setup) — laptop, headset, robot/Orin
4. [Practice in simulation](#4-practice-in-simulation-no-robot)
5. [Real robot session, end to end](#5-real-robot-session-end-to-end)
6. [Controls reference](#6-controls-reference) — Unitree remote, PICO controllers, keyboard
7. [What our commits change vs upstream](#7-what-our-commits-change-vs-upstream)
8. [Troubleshooting](#8-troubleshooting)
9. [Repository layout](#9-repository-layout)

---

## 1. Hardware and network

| Item | Details |
|---|---|
| Robot | Unitree G1 EDU Ultimate, 29 DoF. **Remove the Inspire hands** for full-body sessions (policy trained without distal hand mass). |
| Robot onboard PC | Jetson Orin NX (JetPack 5.1), `unitree@192.168.123.164`. Hosts the `g1-teleop` Wi-Fi AP and the discovery relay. |
| Operator rig | PICO 4 headset (PicoBridge app v0.2.1, `com.picobridge.app`) + 2 PICO Motion Trackers on the ankles. A waist tracker exists but is deliberately unused. |
| Laptop | Ubuntu 22.04, Intel Core Ultra 5, 22 GB RAM, **no NVIDIA GPU**. Runs Teleopit on CPU (torch CPU, ONNX Runtime, MuJoCo). |
| Robot LAN | Ethernet, laptop `192.168.123.2` (NetworkManager profile `robot-lan` on the built-in port, `robot-lan-usb` for a USB dongle). |
| Safety rig | Gantry/tether + a human spotter holding the Unitree remote. **Mandatory.** |

```
                 Wi-Fi "g1-teleop" / teleop12345                Ethernet 192.168.123.0/24
 PICO 4 + trackers ───────────────────────► Orin 10.42.0.1 (NAT) ─────────────────────► Laptop 192.168.123.2
 PicoBridge app                             discovery_relay.py                          Teleopit (this repo)
 (auto-connects)  ◄──── UDP 29888 "192.168.123.2|63901" ◄──┘                            TCP 63901 receiver
                                                                                              │ DDS LowCmd 50 Hz
                                                                                              ▼
                                                                                    G1 motors (via g1_bridge_sdk)
```

Network rules that cost real days when broken:

- **The laptop never joins the `g1-teleop` Wi-Fi.** It has no internet and
  NetworkManager will flap between it and your internet SSID. Laptop Wi-Fi
  stays on the internet network; the headset reaches the laptop through the
  Orin's NAT.
- The headset must **forget every other Wi-Fi** (office, phone hotspot).
  `g1-teleop` has no internet, so the headset otherwise hops away mid-session
  and tracking dies.
- The XRoboToolkit PC service (`holosim-pcservice`, a systemd *user* unit left
  from the previous stack) squats TCP 63901. `go.sh` stops it; if you launch
  by hand run `systemctl --user stop holosim-pcservice` first.
- One commander at a time. If an onboard Teleopit session runs on the Orin,
  the laptop must not start another; `go.sh` checks and refuses.

## 2. Safety rules

1. **Gantry + spotter with the Unitree remote in hand for every powered
   session.** There is no software e-stop you should rely on.
2. **The remote is the e-stop.** Under Teleopit, **L1+R1 = DAMPING** from any
   mode. The robot goes limp immediately, which is a fall if untethered.
   SDK-side damping from the laptop has failed at real emergencies; do not
   build your plan around it.
3. The factory chord **L2+B does nothing while Teleopit commands the robot.**
   Only the Teleopit map in section 6 applies during a session.
4. Software fallback: `bash lab/teleopit_estop.sh` kills the command stream;
   the firmware releases the motors in about 2 s (proven). Tether required.
5. First response to weird motion that is not yet dangerous: **X** (back to
   the self-balancing STANDING pose). Escalate to L1+R1 only if needed.
6. Enter MOCAP only when you stand still in a neutral pose facing the same
   way as the robot. Start with small, slow movements. Never mash buttons.
7. Watch waist_pitch torque and temperature on the first sessions after any
   config change (it once hit a 130 °C fault under the old stack).

## 3. One-time setup

### 3.1 Laptop

```bash
git clone --recurse-submodules git@github.com:mizuharaa/g1-pico-teleop.git Teleopit
cd Teleopit

# Python env (conda; py3.11 is what we run, py3.10+ works)
conda create -n teleopit python=3.11 -y
conda activate teleopit
pip install --upgrade pip
pip install -e '.[pico4]'          # Teleopit + pico-bridge 0.2.1 + sim2real runtime
pip install modelscope
python scripts/setup/download_assets.py --only robots gmr ckpt bvh
#   -> assets/robots/unitree_g1/g1_29dof.xml, ckpt/track_g1.onnx, GMR assets, a sample BVH

# C++ DDS bridge to the G1 (needs Eigen headers; on this laptop conda's were used)
conda install -c conda-forge eigen -y
CPLUS_INCLUDE_PATH=$CONDA_PREFIX/include bash scripts/setup/setup_g1_bridge.sh

# checks
python -c "import teleopit; print('teleopit OK')"
python -c "from pico_bridge import PicoBridge; print('Pico OK')"
python -c "import g1_bridge_sdk; print('bridge OK')"
```

Robot LAN: plug the Ethernet cable and make sure some interface owns
`192.168.123.2` (`ip -br -4 addr`). If not: `nmcli con up robot-lan` (built-in
port) or `nmcli con up robot-lan-usb` (dongle). `go.sh` auto-detects the
interface; manual commands take `real_robot.network_interface=<ifname>`.

Full upstream install matrix (uv/venv, training, review extras):
[`docs/docs/getting-started/installation.md`](docs/docs/getting-started/installation.md).

### 3.2 PICO 4 headset and trackers

1. Developer mode on the headset, USB-C to the laptop, then:
   ```bash
   adb install PicoBridge_v0.2.1.apk       # committed at the repo root
   ```
   (Upstream source of the app:
   [pico-bridge releases](https://github.com/BotRunner64/pico-bridge/releases), v0.2.1.)
2. In headset settings: pair the two PICO Motion Trackers, strap them on the
   **ankles**, enable **full-body tracking**. Calibrate when the headset asks.
3. Wi-Fi: join **`g1-teleop`** (password `teleop12345`), then *forget* all
   other networks. A phone hotspot was needed exactly once for the PicoBridge
   entitlement check; it is cached now.
4. Open **PicoBridge**. The UI is status-only (one slider, nothing else to
   click). It auto-connects when it hears the discovery broadcast; you will see
   it flip to connected a few seconds after the laptop runtime is up.

Trackers go to sleep when still for a while. If the runtime says
*"no body tracking data (wake the ankle trackers)"*, walk a couple of steps and
press Y again.

### 3.3 Robot side (Orin), once

See [`lab/orin/README.md`](lab/orin/README.md): create the `g1-teleop`
hotspot on the Orin (autoconnect) and install
[`lab/orin/discovery_relay.py`](lab/orin/discovery_relay.py) in `unitree`'s
crontab. The relay rebroadcasts the laptop's address into the headset subnet,
which is what makes the headset connect on its own.

### 3.4 Waist lock (check before blaming hardware)

The G1 ships with a **waist lock**: a setting in the Unitree mobile app plus
physical fastener screws. With it engaged, waist_pitch cannot move and the
policy fights the joint ("jam", heat). It was removed on 2026-08-17 and
`teleopit/configs/robot/g1.yaml` now uses the full waist action scale. If the
waist ever stalls again, check the app setting and the screws first; the
rollback is `action_scale[14]: 0.0` in that file.

## 4. Practice in simulation (no robot)

Do this until it is boring. Same headset, same network path, same code; only
the robot is MuJoCo.

```bash
conda activate teleopit && cd ~/Teleopit
systemctl --user stop holosim-pcservice

# (optional) prove the headset stream arrives
python scripts/dev/test_pico_bridge.py --no-video

python scripts/run/run_sim.py --config-name pico4_sim controller.policy_path=ckpt/track_g1.onnx
```

The sim starts in `STANDING`. With the MuJoCo window focused, keyboard **Y**
enters MOCAP (stand still, neutral pose first), **X** returns to STANDING,
**Q** quits. PICO controller **A** pauses/resumes, **B** toggles ARMS mode,
right-stick click toggles the joystick walking mode. `viewers=sim2sim` shows
only the physics view; `viewers=none` is headless.

A robot-side dry run that reads real state but sends no motor commands:

```bash
python scripts/run/standalone_standing.py --policy ckpt/track_g1.onnx --network-interface <ifname> --dry-run
```

## 5. Real robot session, end to end

### Pre-flight (spotter + operator)

- [ ] Robot hanging on the gantry, feet just touching the floor, hands removed.
- [ ] Ethernet from laptop to robot; `ip -br -4 addr` shows `192.168.123.2`.
- [ ] Laptop Wi-Fi on the internet SSID, **not** `g1-teleop`.
- [ ] Unitree remote on and in the spotter's hand. Spotter knows: **L1+R1**.
- [ ] Headset charged, trackers on ankles and awake, headset on `g1-teleop`.
- [ ] Clear floor around the robot; nobody in the swing radius.

### Step 1 — Power the robot and release factory control

Boot the robot as usual (it comes up in the factory controller, damped/zero
torque). On the Unitree remote press **L2+R2** to enter debug/dev mode. This
releases the factory controller so Teleopit's low-level commands are accepted.
Do not press any factory stand-up chord.

### Step 2 — Start the runtime on the laptop

```bash
conda activate teleopit
bash ~/Teleopit/go.sh
```

`go.sh` stops the port squatter, kills stale Teleopit processes, refuses to run
if an onboard session is active, detects the robot interface, launches
`run_sim2real.py` with `pico4_sim2real` and waits for `robot control ready`.
Log: `/tmp/session_monitor/sim2real.log`. Equivalent manual command:

```bash
python scripts/run/run_sim2real.py --config-name pico4_sim2real \
    controller.policy_path=ckpt/track_g1.onnx real_robot.network_interface=<ifname>
```

The robot is now in `IDLE`: state is read, nothing is commanded.

### Step 3 — Headset connects

Put the headset on, open PicoBridge if it is not open. Within ~5 s the app
shows connected and the laptop log starts printing body frames. Operator stands
where they will teleop, facing the same direction as the robot.

### Step 4 — Stand (Unitree remote **Start**)

Press **Start**. Teleopit locks the current joints, then ramps PD gains to the
standing policy over 2 s. The robot balances itself in `STANDING`. Lower the
gantry so the feet carry the weight while the tether stays slack-but-ready.
Let it settle.

### Step 5 — Enter teleop (Unitree remote **Y**)

Operator stands still, neutral pose, aligned with the robot. Spotter presses
**Y**. Teleopit verifies 10 consecutive valid PICO frames, then blends the joint
targets from the standing pose into the operator's pose over 1 s (`MOCAP`).
If it says no tracking data, wake the trackers and press Y again.

Begin with small, slow movements: arms, then torso, then weight shifts, then
steps. The reference is anchored to where the operator stood when Y was
pressed; walking is real displacement (root XY gain 1.25, so the robot
overshoots your displacement slightly).

### Step 6 — During the session

| Want to | Do |
|---|---|
| Freeze the robot in its current pose (bathroom break, re-strap a tracker) | Unitree remote **B** or PICO **A** = pause. Same button resumes. Resume standing still and close to the held pose. |
| Only arms follow, legs/waist hold the standing pose | PICO **B** toggles `ARMS` mode. |
| Walk further than the room allows | PICO **right-stick click** from STANDING or MOCAP toggles `JOYSTICK` mode: left stick forward/back, right stick turn, **grip triggers strafe** (squeeze right grip = strafe right, left grip = left). Caps 0.9 m/s, 0.55 m/s, 1.1 rad/s. **X** exits to STANDING. |
| Switch JOYSTICK → MOCAP | Remote **Y** (or stick-click): the robot goes to STANDING, settles 1.5 s, then prints *"ALIGN YOUR BODY to the robot, then press Y"*. Align, press Y again. Auto-entry was removed after it caused falls. |
| Unexpected motion, not yet dangerous | Remote **X** → STANDING (fast 0.5 s gain ramp back to the standing policy). |
| Emergency | Remote **L1+R1** → DAMPING, robot goes limp on the tether. |

If the PICO stream stops, the robot holds the last reference; it does not
change mode on its own. Use **X**, or **L1+R1** if needed.

### Step 7 — End the session

1. Remote **X** → STANDING. Let it settle.
2. Take the robot's weight on the gantry.
3. Remote **L1+R1** → DAMPING (robot limp on the tether), or `Ctrl+C` /
   `bash lab/teleopit_estop.sh` on the laptop. From DAMPING, **Start** stands
   it up again if you want another round.
4. Stop the laptop runtime (`pkill -f run_sim2real.py`), power the robot down
   per the Unitree procedure, take the headset off `g1-teleop` only if you need
   internet on it.

## 6. Controls reference

### Unitree remote (Teleopit state machine, real robot)

| Button | From | Effect |
|---|---|---|
| **L2+R2** | factory controller | Debug/dev mode: releases factory control so Teleopit can command. Once, after boot. |
| **Start** | IDLE, DAMPING | → STANDING (joint lock, then 2 s Kp ramp into the standing policy). |
| **Y** | STANDING | → MOCAP after 10 valid PICO frames (1 s joint blend-in). Refused with a log message if tracking is not valid. |
| **Y** | JOYSTICK | → STANDING, 1.5 s settle, then waits for an explicit second **Y**. |
| **X** | MOCAP, ARMS, JOYSTICK | → STANDING (0.5 s fast ramp). Also cancels a pending settle. |
| **B** | MOCAP, ARMS | Pause / resume (reference held). |
| **L1+R1** | any | **EMERGENCY DAMPING.** Motors go to damping (kd 8), robot collapses onto the tether. |
| L2+B, Select, A | — | Not used by Teleopit. L2+B (factory damp) is dead while Teleopit runs. |

State machine diagram: [`docs/static/img/diagrams/pico-g1-state-machine.svg`](docs/static/img/diagrams/pico-g1-state-machine.svg).

### PICO 4 controllers (`teleopit/configs/input/pico4.yaml`)

| Control | Effect |
|---|---|
| **A** | Pause / resume the current MOCAP or ARMS session. |
| **B** | Toggle ARMS mode (arms follow, body holds standing pose). |
| **Right stick click** | Toggle JOYSTICK walking mode (from STANDING or MOCAP; goes through a settled STANDING). |
| Left stick up/down | JOYSTICK: forward / back (`vx_max` 0.9 m/s). |
| Right stick left/right | JOYSTICK: turn (`wz_max` 1.1 rad/s). |
| Right grip / left grip | JOYSTICK: strafe right / left (`vy_max` 0.55 m/s). The app never sends left-stick X or right-stick Y, hence the grips. |
| Index triggers | Only used when LinkerHand `hands.mode=gripper` is enabled (not in our setup). |

### Keyboard (simulation only, MuJoCo window focused)

**Y** MOCAP · **X** STANDING · **A** pause/resume · **Q** quit. The Unitree
remote is not read in simulation.

### Factory controller (no Teleopit running)

**L2+B** = damp (never plain B) · **L2+R2** = debug/dev mode. Other factory
chords are in the Unitree manual. Under the previous HoloMotion
stack the map was different (Select = damp, L1 = free fall); that stack is
archived and those rules no longer apply.

## 7. What our commits change vs upstream

`git log f926386..main` (oldest first). All tuning lives in
`teleopit/configs/pico4_sim2real.yaml` and `pico4_sim.yaml`; both files carry
the same values so sim rehearsals match the real robot.

| Commit | Change | Why |
|---|---|---|
| a16e176 | Stale-resume reset, anchor velocity deadbands (soft-knee 0.05 m/s, 0.08 rad/s), smoothing alphas 0.35/0.25 → 0.6/0.5, pilot height 1.74, **waist unlock restore** (`action_scale[14]` 0 → 0.4386) | Sim A/B: no fall, same tracking error, 40 → 20 ms lag; waist lock root-caused as a factory lock, not a fault. |
| f29398a, ed81b2b | `root_xy_gain` 1.143 → 1.25 | Pilot displacement was under-reproduced; overshoot ~9% by request. |
| 46e5cc4, dce4933, 6b174be | **JOYSTICK walking mode** (right-stick click), enterable from STANDING/MOCAP, round-trip toggle | Walk beyond the tracking space. |
| 13fd4f8 | Knee position weight 0/10 → 50 in the PICO→G1 IK; joystick caps raised | Forward kicks were reaching foot targets via hip abduction. |
| 11bbed3 | Reset alignment/velocity on joystick exit; 1.5 s STANDING dwell between mode swaps; knee IK weight 50 → 35 | Stale-yaw lunge collapsed the robot when swapping modes. |
| 9f09761, 3bb24c2 | Strafe moved to grip analogs; clearer "tracker asleep" message; **1 s joint blend-in on MOCAP entry** (`mocap_entry_blend_s`) | Left-stick X / right-stick Y never arrive from the app; entry was a hard pose snap. |
| 8a01ae5 | `foot_z_gain` 1.6; joystick → teleop swap lands in STANDING and requires an explicit Y | Light steps were absorbed by the policy; auto-entry caused falls while the pilot was still in driving posture. |
| (this commit) | `go.sh` interface auto-detect, `lab/` (Orin relay, e-stop, history), this README | Repo now matches the stack that is actually used. |

Tunable knobs with their current values:

| Key (file) | Value | Meaning |
|---|---|---|
| `root_xy_gain` (pico4_sim2real.yaml) | 1.25 | Multiplier on pilot root XY displacement. |
| `anchor_lin_vel_deadband` / `anchor_ang_vel_deadband` | 0.05 / 0.08 | Soft-knee deadbands that kill jitter without eating small steps. |
| `reference_velocity_smoothing_alpha` / `..._anchor_...` | 0.6 / 0.5 | Higher = less lag, more jitter. |
| `mocap_entry_blend_s` | 1.0 | Joint blend from standing pose into the pilot pose on Y. |
| `joystick.*` | vx 0.9, vy 0.55, wz 1.1, deadzone 0.12, settle 1.5 s | Walking-mode caps. |
| `foot_z_gain` (input/pico4.yaml) | 1.6 | Amplifies pilot foot lift height. |
| `human_height` (input/pico4.yaml) | 1.74 | Pilot height, metres. Change per operator. |
| `action_scale[14]` (robot/g1.yaml) | 0.4386 | waist_pitch. 0.0 re-freezes the waist. |
| `runtime.stale_reference_hold_s` / `max_reference_age_s` | 0.08 / 0.25 | Hold last command on stale input; refuse MOCAP entry on an old reference. |

Upstreaming these to BotRunner64/Teleopit is still open.

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `go.sh`: "no interface has a 192.168.123.x address" | Cable, or the NM profile is down: `nmcli con up robot-lan` (built-in port) / `robot-lan-usb` (dongle). Dongle names change with the adapter (`enx…`); never hard-code them. |
| `go.sh`: "an ONBOARD Teleopit session is running" | Someone started Teleopit on the Orin. Kill it there (`pkill -9 -f miniforge3/envs/teleopit`) before running from the laptop. |
| Runtime up, headset never connects | Headset not on `g1-teleop`; relay not running on the Orin (`pgrep -af discovery_relay`); hotspot down (`nmcli con up Hotspot-1` on the Orin); `holosim-pcservice` still holding 63901 (`ss -ltnp \| grep 63901`). |
| Headset connects then drops after a minute | Headset auto-hopped to another Wi-Fi. Forget every other network on it. |
| Y refused: "no body tracking data" | Ankle trackers asleep or full-body tracking off. Walk two steps, check the app status, press Y again. |
| Robot snaps or lunges when entering MOCAP | Pilot was not aligned/still. Use X, align, Y again. The 1 s blend-in only softens joints; the root is anchored at entry. |
| Steps are absorbed, feet barely lift | Raise `foot_z_gain` (1.6 now). |
| Too jittery / too laggy | Deadbands vs smoothing alphas in `pico4_sim2real.yaml`; test in `pico4_sim` first. |
| No LowState from the robot | Wrong interface, DDS on the wrong NIC, or factory control not released (L2+R2). Try `standalone_standing.py --dry-run`. |
| `import g1_bridge_sdk` fails | Rebuild: `git submodule update --init --recursive` then `CPLUS_INCLUDE_PATH=$CONDA_PREFIX/include bash scripts/setup/setup_g1_bridge.sh`. |
| Waist "jams", gets hot | Check the waist lock (app setting + screws) before anything else. Rollback: `action_scale[14]: 0.0`. |
| `pkill`/`pgrep -f` kills the wrong thing or nothing | Self-match trap: the shell command itself contains the pattern. Use the `[t]` bracket trick or a separate shell. |

## 9. Repository layout

```
README.md                  this file (end-to-end operating manual)
README.upstream.md         original Teleopit README (quick start, changelog)
go.sh                      one-command laptop session launcher
PicoBridge_v0.2.1.apk      headset app (adb install)
lab/
  teleopit_estop.sh        software e-stop fallback (kills the command stream)
  orin/discovery_relay.py  headset auto-discovery relay, runs on the Orin
  orin/README.md           Orin hotspot + relay setup
  history/                 PROJECT_STATE / DEVELOPER-HANDOFF from 2026-08-18 (verbatim)
teleopit/                  runtime: inputs (pico4), retargeting (GMR), sim, sim2real (mp runtime, remote parser, safety)
teleopit/configs/          pico4_sim.yaml, pico4_sim2real.yaml, input/pico4.yaml, robot/g1.yaml
scripts/run/               run_sim.py, run_sim2real.py, standalone_standing.py, record_pico_motion.py
scripts/dev/               test_pico_bridge.py, test_g1_bridge.py, benches
scripts/setup/             download_assets.py, setup_g1_bridge.sh
third_party/               g1_bridge_sdk (C++ DDS bridge), unitree_sdk2_python, linkerhand, somehand (submodules)
docs/                      upstream Docusaurus docs (tutorials, configuration reference, architecture)
train_mimic/               upstream training code (not used here; no GPU on the laptop)
```

Not committed (downloaded or generated): `assets/robots/`, `ckpt/`,
`teleopit/retargeting/gmr/assets/`, `data/`, `*.log`, `outputs/`.

Branches on GitHub: `main` (this stack), `archive/holomotion-2026-08-18`
(previous HoloMotion/SONIC workspace with all audits and runbooks).

## License

Apache 2.0 (upstream Teleopit), see [`LICENSE`](LICENSE).
