#!/usr/bin/env bash
# Extra L2+B kill legs for HoloMotion sessions, fired by the g1-dance
# remote killswitch (KILLSWITCH_EXTRA_SH hook):
#   1. LOW-LEVEL DDS damp (g1_estop --now) — works regardless of which
#      controller owns the motors; hardware-proven 2026-08-07.
#   2. Stop the holomotion_g1 container on the Orin so the controller
#      cannot re-command after the damp.
export CYCLONEDDS_HOME="$HOME/.local/cyclonedds"
IFACE=$(ip -4 -o addr | awk '/ 192\.168\.123\./ {print $2; exit}')
PY=~/miniconda3/envs/holomotion_teleop/bin/python
ESTOP=~/full-body-teleoperation/scripts/g1_estop.py
# Order matters (2026-08-10 lockup): a concurrently-dying controller can
# re-stiffen joints AFTER a single damp burst, leaving the robot FROZEN on
# its last command. So: damp, stop the controller, then damp AGAIN so the
# damp command is the last word the motors hear.
"$PY" "$ESTOP" --iface "${IFACE:-enx000ec6c3d44a}" --now
ssh -o BatchMode=yes -o ConnectTimeout=2 unitree@192.168.123.164 \
  "docker stop -t 2 holomotion_g1" >/dev/null 2>&1
"$PY" "$ESTOP" --iface "${IFACE:-enx000ec6c3d44a}" --now
echo "  [extra] damp -> controller stop -> final damp complete"
