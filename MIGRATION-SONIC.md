# SONIC / GR00T-WholeBodyControl migration — state & runbook
(2026-08-11 night. Repo: ~/GR00T-WholeBodyControl. Full feasibility analysis
in the session log; architecture verdict below.)

## Architecture (decided)
- Laptop (no GPU): XRoboToolkit PC service (KEEP the existing systemd
  holosim-pcservice) + `.venv_teleop` (CPU torch) running
  `pico_manager_thread_server.py --manager` + optional MuJoCo sim
  (`run_sim_loop.py`). Headset app → laptop IP, same as always.
- Orin (on robot): the ONLY viable deploy host (`g1_deploy`, TensorRT).
  Laptop cannot run it (no GPU; TRT ~10 GB).
- Thor/CloudXR path: NOT ours (needs Thor backpack). isaacteleop install
  is commented out of install_pico.sh.
- Cloud GPU: training/finetune only (64+ GPUs recommended for finetune;
  NOT needed for stock teleop — released ONNX checkpoints suffice).

## DONE (laptop, 2026-08-11 night)
- Repo cloned + git-lfs installed (~/.local/bin/git-lfs) + LFS pulled for
  external_dependencies/** and gear_sonic/data/**.
- install_pico.sh patched: CPU-only torch (no-GPU laptop), isaacteleop
  skipped. `.venv_teleop` = 2.4 GB, all deps verified (torch 2.13 cpu,
  mujoco 3.11, pinocchio, zmq, xrobotoolkit_sdk built manually after LFS
  fix — build/ dir in external_dependencies/.../build).
- VERIFIED: `run_sim_loop.py` runs (viewer up, waits for DDS peer — the
  benign "lo not multicast-capable" warning is known issue #77 territory).
- VERIFIED: `pico_manager_thread_server.py --manager` connects to our PC
  service and starts its stream server (waits for headset).
- Old stack parked: holosim + holosim-chain systemd units disabled
  (pcservice kept — SONIC uses it). HoloMotion robot image untouched and
  frozen as fallback (tag holomotion:patched-backup-0811 also exists).

## GATE 1 — RETIRED (2026-08-11 late): hardware cleared by running test
Robot runs fine in running/locomotion mode — the harshest joint load there
is. The earlier "knee damage" torque evidence was confounded: the velocity
policy marches in place, so 65-78 Nm knee swings are normal weight
transfer (wrists were a false baseline — they bear no load). Waist thermal
trips are explained by the 08-10 7.7 Hz fault run. Optional: a 15-min
hand-check remains cheap insurance, but it is NOT a gate.
REMAINING root cause for tracking shake: reference-stream teleports
(measured 1.34 m/frame) — a tracking-quality + pipeline issue that the
SONIC migration replaces (plus PICO room hygiene: 50 Hz lock, lighting,
clean IR lenses, tight pants for ankle trackers).

## BACKUPS COMPLETE (2026-08-11 ~17:45) — robot is safe to wipe
All irreplaceables are on the laptop in ~/orin-reflash-backup/:
- holomotion_v1.4.0_orin_jp5.1_arm64.tar (9.1G, md5 7b6d0942 VERIFIED)
- 20260810_170015_{realsense,webcam}.mp4 (fall footage — was robot-only)
- unitree_pc2_factory_backup.tgz (702M: PC2 installers, rtl8852bu Wi-Fi
  driver, unitree_sdk2, cyclonedds_ws, unitree/, Downloads/)
- kc_ws_backup.tgz (1.2G)
- jetpack6/ (the g1-nx-j6.2 image, downloading)
All session videos verified present in ~/Videos/g1-demos/.
The 23G patched HoloMotion image was NOT saved — it is fully re-derivable
(pristine tar + repo deploy scripts + TRT re-bake).

## OVERNIGHT RESULT (2026-08-11 ~21:10) + ONE MORNING DECISION
DONE: g1-nx-j6.2.img.bz2 (3.8G) fully downloaded — the dd image for the
SSD. Integrity check ran overnight (see ping). Everything else staged.
NOT DONE: Jetpack_6.2_nx.tar.bz2 (phase-2 recovery-flash package) is
~10-12G and its extraction needs ~15-20G more — THE LAPTOP CANNOT HOLD IT
alongside the 13G of backups (disk hit 100% at 8.3G partial; partial
deleted to unbrick the system).
MORNING DECISION (pick one):
  A. EXTERNAL USB DRIVE (recommended): plug one in; Claude downloads the
     tar.bz2 straight to it, extracts there, phase-2 flashes from there.
     Backups stay untouched on the laptop.
  B. No drive: approve deleting the 9.1G HoloMotion fallback tar.gz
     (re-downloadable from Horizon's release page) + kc_ws (1.2G) —
     frees ~10G, then a tight download+extract-and-delete dance.
Phase 1 (the dd of the SSD) is fully ready either way — the 3.8G image is
on disk and verified.

## FLASH DAY — physical checklist (user hands; ~30-45 min)
Tools: 5mm T-handle allen (back handle screws), 2mm hex (back cover),
Phillips (NVMe screw), NVMe USB enclosure adapter, USB-C cable.
1. Robot OFF. Remove handle screws -> foam/plastic cover -> NVMe screw ->
   slide SSD out.
2. SSD into USB adapter -> laptop. `sudo umount /dev/sda*`.
3. cd ~/orin-reflash-backup/jetpack6 &&
   `bzip2 -dc g1-nx-j6.2.img.bz2 | sudo dd of=/dev/sda bs=4M
   status=progress conv=fsync`
   (VERIFY /dev/sda is the enclosure first: `lsblk` — do NOT dd the
   laptop's nvme0n1!)
4. `sudo sync && sudo udisksctl power-off -b /dev/sda`. Reinstall SSD.
5. Flashing mode: power on G1, wait 3 steady lights, USB-C to laptop,
   hold BOTH white buttons 2 s, release TOP, hold BOTTOM until 3 green ->
   2 green. Verify with `lsusb` -> must show "NVIDIA Corp. APX".
6. Phase 2 (laptop, in ~/orin-reflash-backup/jetpack6/Jetpack6.2/):
   sudo tar -xjvf Jetpack_6.2_nx.tar.bz2
   cd Jetpack_6.2_nx/Linux_for_Tegra && sudo ./flash_nx_module.sh
   (~8 min; wait for success.)
7. Reassemble (SSD screw, cover, handle). Power on. Then on the Orin:
   sudo nvpmodel -m 0 && sudo jetson_clocks && sudo jetson_clocks --show
   sudo apt-get install -y nvidia-l4t-dla-compiler libcudla-dev-12-6
8. First boot network: ethernet should be 192.168.123.164 (verify; the
   pre-flash identity snapshot is in ~/orin-reflash-backup/robot-identity-
   pre-flash.txt). Find the new default user (check Unitree/NVIDIA docs or
   the login prompt), ssh-copy-id the laptop key, then run
   ~/orin-reflash-backup/orin_bootstrap.sh REMOTELY:
   scp orin_bootstrap.sh <user>@192.168.123.164: && ssh <user>@... 'bash
   orin_bootstrap.sh' (Claude can drive this over ssh once keys work).

## GATE 2 — Orin JetPack reflash (the long pole; USER decision + hands)
Current Orin = JetPack 5.1 (CUDA 11.x era). SONIC REQUIRES JetPack 6.2 +
TensorRT 10.7 EXACTLY (wrong TRT = silently wrong actions = dangerous).
Procedure: docs/source/references/jetpack6.md (NVMe pull + dd of
g1-nx-j6.2.img.bz2 + USB-C recovery flash).
BEFORE FLASHING — BACK UP on the Orin's disk or laptop:
  1. docker save horizonrobotics/holomotion:v1.4.0-orin-jp5.1-arm64 (the
     patched image WITH TRT cache) → tar (the pristine tar already exists
     at ~/holomotion_v1.4.0_orin_jp5.1_arm64.tar).
  2. ~/footage_archive/ (6 session videos live ONLY there now).
  3. Any Unitree factory partitions the jetpack6 doc says to preserve.
NOTE: reflash ends the ability to run the old HoloMotion stack on the
robot until restored. Point of no (easy) return — user must approve.

## After GATE 2 — Orin install (scriptable, ~1-2 h)
1. Clone repo on Orin + git lfs pull.
2. gear_sonic_deploy/scripts/install_deps.sh (aarch64 branch: onnxruntime
   1.16.3, just, nvidia-jetpack).
3. Use JetPack system TRT (/usr/lib/aarch64-linux-gnu/libnvinfer.so),
   verify version == 10.7.
4. source scripts/setup_env.sh && just build.
5. python download_from_hf.py --low-latency && python download_from_hf.py
   (planner_sonic.onnx 774 MB is mandatory); rm -rf ~/.cache/huggingface.
6. python check_environment.py --deploy.
7. sudo systemctl stop iphone_server.service (ZMQ 5557 squatter).

## Bring-up ladder (their procedure, not ours)
1. Sim2sim: laptop `run_sim_loop.py`; Orin `./deploy.sh --cp
   policy/low_latency/model --obs-config policy/low_latency/
   observation_config.yaml <laptop-IP>` (NOT literal 'sim' — that binds
   loopback). Keyboard: ']' then '9', 'T' play motion, 'O' stop.
2. Teleop-in-sim: add `--input-type zmq_manager --zmq-host <laptop-IP>`;
   laptop runs the streamer. Controls: calibration pose → A+B+X+Y engage
   → A+X POSE mode. E-stop: A+B+X+Y (controllers) or 'O' (keyboard).
3. Real robot: same but `real` — ONLY after sim teleop is smooth.
   Requirements: 2 PICO motion trackers ON ANKLES
   (re-pair + calibrate per docs), TIGHT pants (tracker line-of-sight —
   their explicit safety warning), operator at keyboard on 'O'.
4. ALIGN BEFORE MODE SWITCH — their red-box rule: body must match robot
   pose before entering POSE/VR_3PT. (Same physics that bit us all day.)

## Traps that carry over from the old stack
- ONE SDK client per PC service: never run our old holosim-chain (now
  disabled) while the SONIC streamer runs.
- Laptop must NEVER join g1-teleop Wi-Fi (memory: network topology).
- Headset app Mode resets on every reconnect.
- g1-dance killswitch listens on L2+B (Unitree remote) — irrelevant to
  SONIC (PICO controllers), but it still damps the robot if the Unitree
  remote's L2+B is pressed. Decide arm/disarm per session.
