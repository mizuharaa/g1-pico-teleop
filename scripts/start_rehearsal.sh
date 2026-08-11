#!/usr/bin/env bash
# NO-ROBOT REHEARSAL: PICO body tracking -> laptop retarget -> MuJoCo viewer.
# Use this to validate the headset + trackers + calibration before any robot day.
#
# Terminal 1:  ./start_rehearsal.sh          (retarget node; requires PC Service installed:
#                                             sudo dpkg -i ../artifacts/XRoboToolkit_PC_Service_*.deb)
# Terminal 2:  ./start_viewer.sh local       (MuJoCo view of the retargeted G1)
#
# In the headset (same Wi-Fi as this laptop): XRoboToolkit app -> PC Service = laptop IP
# -> WORKING -> enable Head/Controller/Full body/Send.
set -euo pipefail
source ~/miniconda3/etc/profile.d/conda.sh
conda activate holomotion_teleop
cd ~/full-body-teleoperation/HoloMotion/deployment/holomotion_teleop
exec python holomotion_teleop_node.py \
  --robot-zmq-uri "tcp://*:6001" \
  --robot-zmq-mode bind \
  --hz 50 \
  --timing-log-every 250
