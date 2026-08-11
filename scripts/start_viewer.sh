#!/usr/bin/env bash
# Live MuJoCo viewer of the retargeted reference the robot is following.
# Optional — watch-only; dropping frames never affects robot control.
#   ./start_viewer.sh                  -> connect to robot telemetry (port 6002, via Ethernet)
#   ./start_viewer.sh <robot-wifi-ip>  -> same over Wi-Fi
#   ./start_viewer.sh local            -> laptop rehearsal mode (legacy retarget on port 6001)
set -euo pipefail
source ~/miniconda3/etc/profile.d/conda.sh
conda activate holomotion_teleop
cd ~/full-body-teleoperation/HoloMotion/deployment/holomotion_teleop

case "${1:-robot}" in
  local)
    exec python holomotion_teleop_mjviewer.py --uri tcp://127.0.0.1:6001 ;;
  robot)
    exec python holomotion_teleop_mjviewer.py --uri tcp://192.168.123.164:6002 ;;
  *)
    exec python holomotion_teleop_mjviewer.py --uri "tcp://$1:6002" ;;
esac
