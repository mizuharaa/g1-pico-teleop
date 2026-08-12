#!/usr/bin/env bash
# WIRED KILL v2 — the one that actually works in debug mode.
# LocoClient.Damp (old estop_console) talks to the FACTORY service, which is
# RELEASED in debug mode -> old console silently does nothing (2026-08-12).
# This kills the app container instead: firmware loses lowcmd -> damps.
# Robot must be tethered: damp = collapse.
R="unitree@${1:-192.168.123.164}"
echo "=============================================="
echo " G1 WIRED KILL ARMED (docker kill = firmware damp)"
echo " ENTER = KILL APP + DAMP.  Ctrl+C = disarm."
echo "=============================================="
read -r
ssh -o BatchMode=yes -o ConnectTimeout=3 "$R" "docker kill holomotion_g1" \
  && echo "APP KILLED — firmware damping. Robot is collapsing/limp." \
  || echo "SSH FAILED — use spotter/L2+B (killswitch) NOW."
