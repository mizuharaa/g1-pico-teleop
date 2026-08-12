#!/usr/bin/env bash
# SONIC terminal 1: MuJoCo virtual robot (laptop, CPU). Close window to stop.
cd ~/GR00T-WholeBodyControl && source .venv_teleop/bin/activate
# env -u: the old robot stack's CycloneDDS (~/robot/...) leaks in via the
# login env and can shadow the venv's DDS libs (2026-08-12). Loopback DDS
# also needs: sudo ip link set lo multicast on  (done 08-12, not persistent
# across reboots).
# --interface lo: pin sim DDS to loopback — fixes the "create domain error"
# AND guarantees the sim can never talk onto the robot wire (2026-08-12).
exec env -u CYCLONEDDS_HOME -u LD_LIBRARY_PATH python gear_sonic/scripts/run_sim_loop.py --interface lo
