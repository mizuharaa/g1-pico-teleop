#!/usr/bin/env bash
# Arm the L2+B remote killswitch with the HoloMotion extra kill legs.
# Detaches fully (setsid) so terminal/session exits never take it down.
# Verify after: pgrep -f '[r]emote_killswitch.py' + /tmp/killswitch_teleop.log
LOG=/tmp/killswitch_teleop.log
export KILLSWITCH_EXTRA_SH=/home/alois/full-body-teleoperation/scripts/killswitch_extra_holomotion.sh
cd /home/alois/g1-dance || exit 1
: > "$LOG"
echo "[arm_killswitch] launching $(date +%H:%M:%S)" >> "$LOG"
setsid bash deploy/20_remote_killswitch.sh >> "$LOG" 2>&1 < /dev/null &
disown
echo "[arm_killswitch] spawned pgid $!"
