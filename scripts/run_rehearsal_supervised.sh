#!/usr/bin/env bash
# Self-healing rehearsal node. The stock node freezes forever when the PICO
# stream dies (headset off-head, app reconnect) — it just re-publishes the
# last reference. This wrapper watches the node's own rate log and restarts
# the node automatically when the stream stalls, so taking the headset off
# and back on "just works" (~15 s recovery, no manual intervention).
#
# Usage:  ./run_rehearsal_supervised.sh   (PC service must already run,
#         or start it first: /opt/apps/roboticsservice/runService.sh &)
set -uo pipefail
# SINGLE INSTANCE (2026-08-11): repeated launches stacked 3 supervisors + 4
# nodes fighting over the ONE SDK client slot — flock makes seconds a no-op.
exec 200>/tmp/rehearsal_supervisor.lock
flock -n 200 || { echo "[supervisor] another instance holds the lock — exiting"; exit 0; }
source ~/miniconda3/etc/profile.d/conda.sh
conda activate holomotion_teleop
cd ~/full-body-teleoperation/HoloMotion/deployment/holomotion_teleop

graceful_kill() {
    # SIGTERM first so the node's handler runs stop() -> xrt.close(); a
    # SIGKILL leaks the PC-service SDK client slot (one client max!) and the
    # next node connects but never receives body data. Escalate only if the
    # node ignores TERM for 3 s.
    local p="$1"
    kill "$p" 2>/dev/null || return 0
    for _ in 1 2 3; do
        kill -0 "$p" 2>/dev/null || return 0
        sleep 1
    done
    kill -9 "$p" 2>/dev/null
}

kill_stray_nodes() {
    # exactly one node may exist (ours). Kill by PID list — pattern pkill
    # self-matches shell command lines and has burned us repeatedly.
    for p in $(pgrep -f teleop_node.py); do
        [ "$p" != "${NODE:-x}" ] && graceful_kill "$p"
    done
}
kill_stray_nodes

LOG="${REHEARSAL_LOG:-/tmp/rehearsal_node.log}"
STALL_S=12   # stream considered dead after this many seconds without frames
STALL_STREAK=0   # consecutive no-data node restarts (set -u: must init)

echo "[supervisor] log: $LOG  (stall threshold ${STALL_S}s)"
VIEWER_LOG="${VIEWER_LOG:-/tmp/rehearsal_viewer.log}"

restart_viewer() {
    # NO_VIEWER=1: skip the reference viewer entirely (e.g. when the simsmoke
    # interactive window is the display — two MuJoCo windows confused everyone
    # on 2026-08-11).
    if [ "${NO_VIEWER:-0}" = "1" ]; then
        return 0
    fi
    pkill -9 -f holomotion_teleop_mjviewer 2>/dev/null
    sleep 1
    DISPLAY="${DISPLAY:-:0}" PYTHONUNBUFFERED=1 python holomotion_teleop_mjviewer.py \
        --uri tcp://127.0.0.1:6001 >> "$VIEWER_LOG" 2>&1 &
    echo "[supervisor] viewer restarted (pid $!)"
}

# PC Service must exist from second zero — not only between node cycles
systemctl --user start holosim-pcservice.service 2>/dev/null || true

# viewer: start ONCE; it survives node restarts (ZMQ auto-reconnects).
# Only relaunch it if its process actually dies — no window flapping.
restart_viewer

while true; do
    kill_stray_nodes
    : > "$LOG"
    # 200>&- : do NOT inherit the flock fd — an orphaned node holding it
    # would block every future supervisor forever ("another instance" loop).
    # reference tape (2026-08-12 decisive-run protocol): one file per node
    # incarnation; saved by the node's SIGTERM handler on every bounce.
    mkdir -p ~/ref_tapes
    TAPE=~/ref_tapes/ref_$(date +%Y%m%d_%H%M%S).npz
    echo "[supervisor] reference tape -> $TAPE"
    PYTHONUNBUFFERED=1 python holomotion_teleop_node.py \
        --robot-zmq-uri "tcp://*:6001" --robot-zmq-mode bind \
        --hz 50 --timing-log-every 250 --skip-start-service \
        --debug-retarget-dump "$TAPE" \
        >> "$LOG" 2>&1 200>&- &
    NODE=$!
    echo "[supervisor] node started (pid $NODE)"
    # app-connection fingerprint: a NEW headset/app TCP session to the PC
    # service means the operator hit Send/reconnect — bounce the node so a
    # fresh SDK session picks the new stream up immediately (the SDK latch
    # otherwise leaves the old session deaf; 2026-08-11).
    CONNS_PREV=$(ss -tn state established '( sport = :63901 )' 2>/dev/null | tail -n +2 | awk '{print $5}' | sort | tr '\n' ' ')
    pgrep -f "mjviewer" >/dev/null || restart_viewer

    # health loop: node is healthy while PicoReader lines keep appearing
    LAST_OK=$(date +%s)
    SEEN_DATA=${SEEN_DATA:-0}
    while kill -0 "$NODE" 2>/dev/null; do
        sleep 3
        CONNS=$(ss -tn state established '( sport = :63901 )' 2>/dev/null | tail -n +2 | awk '{print $5}' | sort | tr '\n' ' ')
        NEW_PEER=""
        for c in $CONNS; do
            case " $CONNS_PREV " in *" $c "*) ;; *) NEW_PEER="$c";; esac
        done
        CONNS_PREV="$CONNS"
        if [ -n "$NEW_PEER" ]; then
            echo "[supervisor] NEW app connection ($NEW_PEER) -> refreshing node for a clean SDK session"
            # graceful (2026-08-12): SIGTERM lets the node save its reference
            # tape AND close the SDK client slot (kill -9 leaked it — see
            # graceful_kill comment; the transition tape is the evidence we
            # need when a violent run coincides with a reconnect bounce).
            graceful_kill "$NODE"
            wait "$NODE" 2>/dev/null
            break
        fi
        # one pattern for gate AND extraction (the old pair could pass the
        # gate on one line and read the rate from another)
        RATE=$(tail -40 "$LOG" | grep -oE 'actual=[0-9]+\.[0-9]+Hz' | tail -1 | tr -dc '0-9.')
        # floor 15 Hz: catches wedged/near-dead loops (frozen SDK latch runs
        # 0-2 Hz) without restart-thrashing on a Wi-Fi dip — a node restart
        # cannot fix a slow-but-alive stream, it only churns the SDK client.
        if [ -n "$RATE" ] && [ "${RATE%%.*}" -ge 15 ] 2>/dev/null; then
            LAST_OK=$(date +%s)
            STALL_STREAK=0   # real data seen -> not a headset-side outage
            SEEN_DATA=1
        fi
        # re-arm the grace period on positive evidence of an app-side outage
        # (headset sleep): the operator needs 60-90 s to re-wear + re-tick
        # Send/Mode; a 12 s kill loop makes that recovery unwinnable.
        if tail -20 "$LOG" 2>/dev/null | grep -q "head_pose_dead\|timestamps FROZEN"; then
            SEEN_DATA=0
        fi
        NOW=$(date +%s)
        # startup grace (2026-08-11 audit): before the FIRST data the 12 s
        # kill-loop tore down the SDK handshake faster than an operator can
        # finish headset setup. 90 s grace until data has been seen once.
        THRESH=$STALL_S; [ "${SEEN_DATA:-0}" = 1 ] || THRESH=90
        if [ $((NOW - LAST_OK)) -ge $THRESH ]; then
            echo "[supervisor] stream stalled ${THRESH}s -> restarting node"
            graceful_kill "$NODE"
            wait "$NODE" 2>/dev/null
            STALL_STREAK=$((STALL_STREAK + 1))
            break
        fi
    done
    wait "$NODE" 2>/dev/null   # reap self-exited nodes too
    # PC Service health check on every respawn cycle. NEVER kill a service
    # that is listening (the headset may be mid-handshake with it — blind
    # recycling caused "connecting... failed" loops on 2026-08-10); only
    # (re)start when port 63901 has no listener. setsid fully detaches it
    # so supervisor restarts can't take it down.
    SERVICE_RECYCLE=""
    if ! ss -tln 2>/dev/null | grep -q ':63901 '; then
        SERVICE_RECYCLE="not listening on 63901"
    elif [ "${STALL_STREAK:-0}" -ge 2 ] && \
         [ "$(tail -120 "$LOG" 2>/dev/null | grep -c head_pose_dead)" -ge 3 ] && \
         ! tail -120 "$LOG" 2>/dev/null | grep -q "head_pose_ok"; then
        # LOCAL PATCH (2026-08-11 rev2): recycle ONLY on POSITIVE evidence of
        # a dead link (>=3 head_pose_dead probes, zero head_pose_ok). The
        # first version keyed on the ABSENCE of head_pose_ok — which also
        # matched "diagnostic not printing yet" and recycled the service
        # while the operator was mid-connect, resetting the app's Mode each
        # time. Never again: if head_pose_ok appears even once, hands off.
        SERVICE_RECYCLE="listening but WEDGED (3+ dead head-pose probes, ${STALL_STREAK} dataless node restarts)"
    fi
    if [ -n "$SERVICE_RECYCLE" ]; then
        echo "[supervisor] PC Service $SERVICE_RECYCLE -> recycling it"
        systemctl --user restart holosim-pcservice.service 2>/dev/null || {
            pkill -9 -f RoboticsServiceProcess 2>/dev/null; sleep 1
            (cd /opt/apps/roboticsservice && \
             setsid nohup bash runService.sh >> /tmp/pc_service.log 2>&1 < /dev/null &)
        }
        sleep 3
        if ss -tln 2>/dev/null | grep -q ':63901 '; then
            echo "[supervisor] PC Service is back up — RE-CONNECT the app (192.168.123.2) and re-set Mode=Full-body"
            STALL_STREAK=0
        else
            echo "[supervisor] WARN: PC Service still not listening — check /tmp/pc_service.log"
        fi
    elif tail -80 "$LOG" 2>/dev/null | grep -q "head_pose_ok"; then
        echo "[supervisor] headset link is LIVE — missing body data is app-side:"
        echo "             set Mode=Full-body in the app + 3 trackers lit. NOT restarting anything."
    fi
    if [ "${STALL_STREAK:-0}" -ge 3 ]; then
        echo "=============================================================="
        echo "[supervisor] node restarted ${STALL_STREAK}x with no body data."
        echo "  A laptop-side restart CANNOT fix this. On the HEADSET:"
        echo "  1. WEAR it (off-head = sleep = no tracking, no data)"
        echo "  2. Check Wi-Fi — it flees to other networks after AP sleep"
        echo "  3. XRoboToolkit: PC Service status must be WORKING"
        echo "  4. Re-tick Head + Controller + Send, and Mode -> Full-body"
        echo "     (the app RESETS Mode to None on every reconnect!)"
        echo "  5. All 3 trackers on and lit (Num must show 3)"
        echo "=============================================================="
    fi
    echo "[supervisor] node exited/killed; respawning in 2 s (Ctrl+C to stop)"
    sleep 2
done
