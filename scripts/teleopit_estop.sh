#!/usr/bin/env bash
# Emergency stop for Teleopit sessions (2026-08-14).
#
# PROVEN mechanism (14:32 ankle-chatter event): killing the process that
# streams LowCmd makes the G1 firmware release the motors within ~2 s.
# The LocoClient.Damp SDK call does NOT work while the bridge holds the
# robot (three DAMP UNCONFIRMED events on record) — it is attempted last,
# best-effort only. Robot must be tethered or a fall is expected.
for p in $(pgrep -f "miniconda3/envs/teleopi[t]/bin/python"); do
  kill -9 "$p" 2>/dev/null
done
echo "[teleopit_estop] command stream killed -> firmware damp expected in ~2s"
# best-effort SDK damp (works only when nothing else commands the robot)
CYCLONEDDS_HOME=$HOME/.local/cyclonedds \
  $HOME/miniconda3/envs/holomotion_teleop/bin/python \
  $HOME/full-body-teleoperation/scripts/g1_estop.py --iface enx000ec6c3d44a --now
