#!/usr/bin/env bash
# One-command Teleopit session launcher (2026-08-14).
#   bash ~/Teleopit/go.sh
# Cleans stale processes/ports, starts sim2real, waits for ready.
# The PicoBridge app connects BY ITSELF ~5 s after this prints READY.
set -u
cd "$(dirname "$0")"

# GUARD (2026-08-14): never allow two commanders. If an onboard session is
# running on the Orin, this laptop launcher must not start a second one —
# the robot obeys whichever stream persists and no remote button can win.
if ssh -o BatchMode=yes -o ConnectTimeout=4 unitree@192.168.123.164 \
     'pgrep -f "miniforge3/envs/teleopi[t]/bin/python" >/dev/null' 2>/dev/null; then
  echo "ABORT: an ONBOARD Teleopit session is running on the robot."
  echo "Kill it first:  ssh unitree@192.168.123.164 'pkill -9 -f miniforge3/envs/teleopit'"
  exit 1
fi

# free the receiver port: old teleopit processes + the XRoboToolkit PC service
systemctl --user stop holosim-pcservice 2>/dev/null
for p in $(pgrep -f "miniconda3/envs/teleopi[t]/bin/python"); do kill -9 "$p" 2>/dev/null; done
sleep 2

LOG=/tmp/session_monitor/sim2real.log
mkdir -p /tmp/session_monitor
setsid nohup "$HOME/miniconda3/envs/teleopit/bin/python" -u scripts/run/run_sim2real.py \
    --config-name pico4_sim2real \
    controller.policy_path=ckpt/track_g1.onnx \
    real_robot.network_interface=enx000ec6c3d44a \
    > "$LOG" 2>&1 < /dev/null &

for i in $(seq 1 30); do
  sleep 1
  if grep -q "robot control ready" "$LOG" 2>/dev/null; then
    echo "READY — mode IDLE. Headset app connects itself in ~5 s."
    echo "Remote: Start=stand  Y=mocap  X=stand  B/PicoA=pause  L1+R1=DAMP(falls if untethered)"
    exit 0
  fi
  if grep -qE "Error|Traceback" "$LOG" 2>/dev/null; then
    echo "FAILED — last log lines:"; tail -5 "$LOG"; exit 1
  fi
done
echo "TIMEOUT waiting for ready — tail $LOG"; exit 1
