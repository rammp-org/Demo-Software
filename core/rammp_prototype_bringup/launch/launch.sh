#!/bin/bash
# launch.sh — RAMMP full system launcher
# Usage:   ./launch.sh [OPTIONS]
# Kill:    Ctrl+C
#
# Options (passed through to jetson launch file):
#   --no-cameras          skip camera nodes
#   --no-arm              skip arm driver
#   --no-luci             skip LUCI chair interface
#   --serial-port PORT    override serial port (default: /dev/ttyACM0)
#   --chair-ip IP         override chair IP (default: 10.2.10.3)
#   --neu-navigation      enable NEU curb detection

set -e

# ── Config ─────────────────────────────────────────────────────────────────────
LAPTOP_USER="rammp"
LAPTOP_IP="192.168.1.13"

JETSON_WS="$HOME/ros2_ws"
LAPTOP_WS="/home/$LAPTOP_USER/ros2_ws"

ROS_SETUP="/opt/ros/humble/setup.bash"
SESSION="rammp"

# Default launch arguments (mirrors your launch file defaults)
SERIAL_PORT="/dev/ttyACM0"
CHAIR_IP="10.2.10.3"
UE_HOST="127.0.0.1"
LAUNCH_CAMERAS="true"
LAUNCH_ARM="true"
LAUNCH_LUCI="true"
LAUNCH_NEU="false"
# ───────────────────────────────────────────────────────────────────────────────

# ── Parse CLI flags ────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-cameras)         LAUNCH_CAMERAS="false" ;;
        --no-arm)             LAUNCH_ARM="false" ;;
        --no-luci)            LAUNCH_LUCI="false" ;;
        --neu-navigation)     LAUNCH_NEU="true" ;;
        --serial-port)        SERIAL_PORT="$2"; shift ;;
        --chair-ip)           CHAIR_IP="$2"; shift ;;
        --ue-host)            UE_HOST="$2"; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

# ── Cleanup (runs on Ctrl+C or any exit) ──────────────────────────────────────
cleanup() {
    echo ""
    echo "Shutting down RAMMP system..."

    # Kill laptop node first (remote)
    ssh "$LAPTOP_USER@$LAPTOP_IP" \
        "pkill -f 'ros2 launch' || pkill -f 'ros2 run' || true" 2>/dev/null \
        && echo "  Laptop nodes stopped." \
        || echo "  (Could not reach laptop — may already be down.)"

    # Kill the whole tmux session — terminates all local panes
    tmux kill-session -t "$SESSION" 2>/dev/null || true

    echo "Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── Preflight checks ───────────────────────────────────────────────────────────
echo "Running preflight checks..."

# Check SSH reachability
if ! ssh -o ConnectTimeout=4 "$LAPTOP_USER@$LAPTOP_IP" exit 2>/dev/null; then
    echo "ERROR: Cannot reach laptop at $LAPTOP_IP. Check ethernet connection."
    exit 1
fi
echo "  Laptop reachable."

# Check serial port (if mebot driver will be launched)
if [[ ! -e "$SERIAL_PORT" ]]; then
    echo "WARNING: Serial port $SERIAL_PORT not found. MEBot driver may fail."
fi

# Kill any leftover session from a previous run
tmux kill-session -t "$SESSION" 2>/dev/null || true

# ── Build the launch argument string ──────────────────────────────────────────
JETSON_ARGS=(
    "serial_port:=$SERIAL_PORT"
    "chair_ip:=$CHAIR_IP"
    "ue_host:=$UE_HOST"
    "launch_cameras:=$LAUNCH_CAMERAS"
    "launch_arm_driver:=$LAUNCH_ARM"
    "launch_luci:=$LAUNCH_LUCI"
    "launch_neu_navigation:=$LAUNCH_NEU"
)
ARGS_STR="${JETSON_ARGS[*]}"

# ── Launch ─────────────────────────────────────────────────────────────────────
echo "Starting RAMMP system..."
echo "  Serial:  $SERIAL_PORT"
echo "  Chair:   $CHAIR_IP"
echo "  UE host: $UE_HOST"
echo "  Laptop:  $LAPTOP_USER@$LAPTOP_IP"
echo ""

# Window 1: Jetson — all local nodes via your existing launch file
tmux new-session -d -s "$SESSION" -n "jetson" \
    "bash -c 'source $ROS_SETUP && \
              source $JETSON_WS/install/setup.bash && \
              ros2 launch rammp_prototype_bringup full_system.launch.py $ARGS_STR; \
              echo \"[jetson] Launch exited.\"; read'"

# Window 2: Laptop node (SSH)
tmux new-window -t "$SESSION" -n "laptop" \
    "bash -c 'ssh $LAPTOP_USER@$LAPTOP_IP \
        \"source $ROS_SETUP && \
         source $LAPTOP_WS/install/setup.bash && \
         ros2 launch your_pkg laptop.launch.py\"; \
     echo \"[laptop] SSH session ended.\"; read'"

# Window 3: Monitor — handy to have open immediately
tmux new-window -t "$SESSION" -n "monitor" \
    "bash -c 'source $ROS_SETUP && \
              source $JETSON_WS/install/setup.bash && \
              echo \"Monitor ready. Try: ros2 topic list, ros2 node list\"; \
              bash'"

# Start on the jetson window
tmux select-window -t "$SESSION:jetson"

echo "Attaching (Ctrl+C to kill everything, Ctrl+B + number to switch windows)..."
tmux attach-session -t "$SESSION"
