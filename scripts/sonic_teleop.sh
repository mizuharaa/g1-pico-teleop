#!/usr/bin/env bash
# SONIC terminal 2: PICO streamer (laptop). Headset app -> THIS laptop's IP
# (192.168.123.2 on the wire / whatever wlan IP), Send + Full body + ankle
# trackers. First run: add --vis_vr3pt --vis_smpl to see the reference.
# NOTE: our old holosim-chain must stay OFF (one SDK client per PC service).
systemctl --user is-active holosim-chain 2>/dev/null | grep -q active && {
  echo "FATAL: old holosim-chain running — systemctl --user stop holosim-chain"; exit 1; }
cd ~/GR00T-WholeBodyControl && source .venv_teleop/bin/activate
exec python gear_sonic/scripts/pico_manager_thread_server.py --manager "$@"
