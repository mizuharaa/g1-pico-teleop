#!/usr/bin/env bash
# (Re)arm the joint watchdog with re-arming auto-damp. Needs the wired link.
export CYCLONEDDS_HOME="$HOME/.local/cyclonedds"
PY=~/miniconda3/envs/holomotion_teleop/bin/python
S=~/full-body-teleoperation/scripts
for p in $(pgrep -f 'joint_watchdog\.py'); do kill -9 "$p" 2>/dev/null; done
sleep 1
IFACE=$(ip -4 -o addr | awk '/ 192\.168\.123\./ {print $2; exit}')
[ -z "$IFACE" ] && { echo "no robot LAN — plug the cable"; exit 1; }
setsid nohup "$PY" "$S/joint_watchdog.py" --iface "$IFACE" \
  --csv "/tmp/session_monitor/watchdog_$(date +%H%M%S).csv" \
  --on-alarm "$PY $S/g1_estop.py --iface $IFACE --now" \
  > /tmp/watchdog_teleop.log 2>&1 < /dev/null &
disown
sleep 4
pgrep -f 'joint_watchdog\.py' >/dev/null && echo "watchdog ARMED on $IFACE" || \
  { echo "watchdog failed:"; tail -3 /tmp/watchdog_teleop.log; }
