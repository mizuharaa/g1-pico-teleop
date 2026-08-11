#!/usr/bin/env bash
# One command: XR headset -> laptop PC service -> CPU retarget (self-healing
# supervisor) -> zmq -> interactive MuJoCo sim of the full deployed stack.
#
#   ~/full-body-teleoperation/bench/simsmoke/sim_with_pico.sh
#
# Headset network: headset joins 'g1-teleop' (robot AP, robot powered on);
# the robot NATs it onto the wire. In the headset app enter 192.168.123.2
# (this laptop's ethernet IP) — NOT 10.42.0.1, and the laptop's Wi-Fi
# stays on normal internet.
# Headset off/on or app reconnect: supervisor restarts the retarget node
# (~15 s), sim falls back to velocity stand — press 3 (B) to resume tracking.
set -uo pipefail
D=~/full-body-teleoperation
cd "$D/bench/simsmoke"

# ONE sim only. holosim.service may already run this whole chain at login —
# two ONNX+MuJoCo 50 Hz loops starve each other and both miss realtime, and
# their keepers fight over the heartbeat/relaunch (confirmed 08-11 flakiness
# source). Stop the unit first if you want a manual run.
# exact-match: pgrep -f also matches OTHER processes whose cmdline merely
# CONTAINS the string (monitors/wrappers) — false-FATALed on 2026-08-11
if ps -eo cmd | grep -q '^python interactive\.py'; then
  echo "FATAL: a sim is already running (holosim.service? another terminal?)"
  echo "  stop it first:  systemctl --user stop holosim.service; pkill -f 'python interactive.py'"
  exit 1
fi

# smoothness knobs — DEFAULT OFF everywhere (2026-08-11 evening): they were
# validated only against the synthetic suite reference, NOT the live pico
# path, and stacking them coincided with tracking falls. Opt in explicitly
# (HOLOMOTION_TARGET_SLEW_RAD_S=15, HOLOTELEOP_REF_SMOOTH=0.4) and A/B one
# at a time against the live headset before trusting either.
export HOLOMOTION_TARGET_SLEW_RAD_S="${HOLOMOTION_TARGET_SLEW_RAD_S:-0}"
export HOLOTELEOP_REF_SMOOTH="${HOLOTELEOP_REF_SMOOTH:-0}"

echo "== laptop IPs (enter one of these in the headset app) =="
ip -4 -o addr | awk '{print "  "$2": "$4}' | grep -v "127.0.0.1"

echo "== headset network path (laptop Wi-Fi is NOT touched — stays on internet) =="
# Topology: headset on 'g1-teleop' (robot AP) -> robot NATs 10.42.0.0/24 ->
# laptop ethernet 192.168.123.2 (PC service). Verified 2026-08-11.
# NEVER join this laptop to g1-teleop: that AP has no internet and the
# roaming broke everything on 08-11.
if ping -c1 -W2 192.168.123.164 >/dev/null 2>&1; then
  echo "  robot reachable — headset app must use IP: 192.168.123.2"
else
  echo "  WARN: robot unreachable — power it on (its AP carries the headset)"
fi

echo "== PC service =="
if pgrep -f RoboticsServiceProcess >/dev/null; then
  echo "  already running"
elif systemctl start roboticsservice 2>/dev/null && sleep 2 && pgrep -f RoboticsServiceProcess >/dev/null; then
  echo "  started (systemd)"
else
  # runService.sh uses bashisms + relative exec path: needs bash + its own cwd
  (cd /opt/apps/roboticsservice && nohup bash runService.sh > /tmp/pico_service_sim.log 2>&1 &)
  sleep 2
  pgrep -f RoboticsServiceProcess >/dev/null && echo "  started" || {
    echo "  FAILED to start — check /tmp/pico_service_sim.log"; exit 1; }
fi

echo "== retarget supervisor (self-healing; ONE window only — no reference viewer) =="
if pgrep -f run_rehearsal_supervised >/dev/null; then
  echo "  already running"
else
  # bash prefix: a lost execute bit must fail loudly, not kill the chain
  # silently (bit was lost by an edit at 13:53 on 08-11 — cost a session)
  NO_VIEWER=1 setsid nohup bash "$D/scripts/run_rehearsal_supervised.sh" > /tmp/rehearsal_supervisor.log 2>&1 < /dev/null &
  sleep 2
  if pgrep -f run_rehearsal_supervised >/dev/null; then
    echo "  started (log: /tmp/rehearsal_supervisor.log)"
  else
    echo "  FAILED to start — see /tmp/rehearsal_supervisor.log:"
    tail -3 /tmp/rehearsal_supervisor.log
    exit 1
  fi
fi

echo "== interactive sim (close window or ESC to quit; auto-relaunch on crash/freeze/sleep) =="
source ~/miniconda3/etc/profile.d/conda.sh
conda activate holomotion_teleop
# per-instance heartbeat: a shared path let one sim's heartbeat satisfy the
# other keeper's freeze watchdog (and each keeper rm -f'd the other's file)
HB=/tmp/holosim_heartbeat.$$
export HOLOSIM_HEARTBEAT="$HB"
trap 'rm -f "$HB"' EXIT
while true; do
  rm -f "$HB"
  DISPLAY="${DISPLAY:-:0}" python interactive.py --fast --pico &
  SIM=$!
  # freeze watchdog: process alive but heartbeat stale >20 s -> kill+relaunch
  while kill -0 "$SIM" 2>/dev/null; do
    sleep 5
    if [ -f "$HB" ]; then
      AGE=$(( $(date +%s) - $(date -r "$HB" +%s 2>/dev/null || date +%s) ))
      if [ "$AGE" -gt 20 ]; then
        echo "[keeper] sim frozen ${AGE}s -> relaunching"
        kill -9 "$SIM" 2>/dev/null
        break
      fi
    fi
  done
  wait "$SIM" 2>/dev/null
  CODE=$?
  if [ "$CODE" -eq 0 ]; then
    echo "[keeper] sim quit deliberately — not relaunching"
    break
  fi
  echo "[keeper] sim died (code $CODE) -> relaunching in 3 s"
  sleep 3
done

echo
echo "sim closed. Retarget supervisor + PC service left running for quick"
echo "relaunch; stop them with:  pkill -f run_rehearsal_supervised; pkill -f holomotion_teleop_node; pkill -f roboticsservice"
