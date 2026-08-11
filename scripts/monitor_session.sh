#!/usr/bin/env bash
# FIRST-RUN SESSION MONITOR — one terminal showing everything that matters.
#
#   ~/full-body-teleoperation/scripts/monitor_session.sh
#
# - Rolling dual-camera snapshots (operator + robot) every 10 s into
#   /tmp/session_monitor/ (latest.jpg + last ~10 min timestamped history,
#   for post-incident review).
# - Aggregated live tail of every laptop-side log (killswitch, rehearsal
#   node, supervisor, viewer, PC service).
# - When the robot LAN is up, also tails the HoloMotion container log on
#   the Orin over ssh.
set -uo pipefail
MON=/tmp/session_monitor
mkdir -p "$MON"
PY=~/miniconda3/envs/holomotion_teleop/bin/python
SNAP=~/full-body-teleoperation/scripts/session_snap.py

# ---- camera loop (background) -------------------------------------------
(
  while :; do
    # live_cam.py owns the camera when running (UVC = single consumer)
    # and writes the rolling snapshots itself — skip to avoid fighting it.
    if ! pgrep -f "python.*live_cam.py" >/dev/null 2>&1; then
      TS=$(date +%H%M%S)
      "$PY" "$SNAP" "$MON/snap_$TS.jpg" >/dev/null 2>&1 && \
        cp "$MON/snap_$TS.jpg" "$MON/latest.jpg" 2>/dev/null
      ls -t "$MON"/snap_*.jpg 2>/dev/null | tail -n +61 | xargs -r rm -f
    fi
    sleep 10
  done
) &
CAM_PID=$!
trap 'kill $CAM_PID 2>/dev/null' EXIT
echo "[monitor] camera snapshots -> $MON/latest.jpg (10 s cadence, 10 min history)"

# ---- robot container log (background, best-effort) ----------------------
(
  while :; do
    if ping -c1 -W1 192.168.123.164 >/dev/null 2>&1; then
      echo "[monitor] robot LAN up — tailing Orin container log"
      ssh -o ConnectTimeout=3 -o BatchMode=yes unitree@192.168.123.164 \
        'tail -F /tmp/holomotion_pico_service.log 2>/dev/null' 2>/dev/null \
        | sed 's/^/[ORIN] /'
      echo "[monitor] Orin log tail ended; retrying in 10 s"
    fi
    sleep 10
  done
) &
ORIN_PID=$!
trap 'kill $CAM_PID $ORIN_PID 2>/dev/null' EXIT

# ---- aggregated laptop logs (foreground) --------------------------------
echo "[monitor] tailing laptop logs (Ctrl+C stops the monitor only)"
touch /tmp/killswitch_teleop.log /tmp/rehearsal_node.log \
      /tmp/rehearsal_supervisor.log /tmp/rehearsal_viewer.log /tmp/pc_service.log
exec tail -F /tmp/killswitch_teleop.log /tmp/rehearsal_node.log \
     /tmp/rehearsal_supervisor.log /tmp/rehearsal_viewer.log /tmp/pc_service.log
