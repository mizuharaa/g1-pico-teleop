#!/usr/bin/env bash
# Restart the external-cam viewer/snapshotter (safe against pgrep self-match:
# this script's own cmdline contains neither pattern).
for p in $(pgrep -f 'live_cam\.py'); do kill -9 "$p" 2>/dev/null; done
sleep 1
DISPLAY=:0 setsid nohup ~/miniconda3/envs/holomotion_teleop/bin/python \
  ~/full-body-teleoperation/scripts/live_cam.py \
  > /tmp/live_cam.log 2>&1 < /dev/null &
disown
echo "cam restarted"
