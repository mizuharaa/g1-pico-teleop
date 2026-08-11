#!/usr/bin/env bash
# Record a demo with BOTH cameras at 1080p30 (parks the live viewer, which
# shares the RealSense, and restores it afterwards).
#   demo_record.sh                # record until q / Ctrl+C, with preview
#   demo_record.sh 60             # record 60 seconds
# Files land in ~/Videos/g1-demos/
for p in $(pgrep -f 'live_cam\.py'); do kill "$p" 2>/dev/null; done
sleep 1
DUR="${1:-0}"
[ $# -gt 0 ] && shift
DISPLAY="${DISPLAY:-:0}" ~/miniconda3/envs/holomotion_teleop/bin/python \
  ~/full-body-teleoperation/scripts/demo_record.py --preview --duration "$DUR" "$@"
bash ~/full-body-teleoperation/scripts/restart_cam.sh
