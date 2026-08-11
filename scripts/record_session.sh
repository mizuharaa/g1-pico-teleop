#!/usr/bin/env bash
# Start/stop the SESSION TAPE: background dual-camera recording (no preview),
# runs until stopped. The recorder keeps the monitor snapshot alive itself.
#   record_session.sh start [--smooth]
#   record_session.sh stop
S=~/full-body-teleoperation/scripts
case "${1:-start}" in
  stop)
    pkill -INT -f 'demo_record\.py' 2>/dev/null && echo "stopping recorder..."
    sleep 3
    ls -t ~/Videos/g1-demos/*.mp4 2>/dev/null | head -2
    bash "$S/restart_cam.sh"
    ;;
  start)
    shift
    for p in $(pgrep -f 'live_cam\.py'); do kill "$p" 2>/dev/null; done
    for p in $(pgrep -f 'demo_record\.py'); do kill "$p" 2>/dev/null; done
    sleep 1
    DISPLAY="${DISPLAY:-:0}" setsid nohup ~/miniconda3/envs/holomotion_teleop/bin/python \
      "$S/demo_record.py" "$@" > /tmp/demo_record.log 2>&1 < /dev/null &
    disown
    sleep 4
    grep -a recording /tmp/demo_record.log || { echo "recorder failed:"; tail -3 /tmp/demo_record.log; }
    ;;
esac
