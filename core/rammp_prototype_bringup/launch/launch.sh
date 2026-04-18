#!/bin/bash
# launch.sh — RAMMP full system launcher
# Usage:   ./launch.sh [OPTIONS]
# Kill:    Ctrl+C
#
# Options (passed through to jetson launch file):
#   --no-cameras          skip camera nodes
#   --no-arm              skip arm driver
#   --no-luci             skip LUCI chair interface
#   --serial-port PORT    override serial port (default: /dev/ttyACM0)
#   --chair-ip IP         override chair IP (default: 10.2.10.3)
#   --neu-navigation      enable NEU curb detection

# TODO: add this back
# set -e

# ── Config ─────────────────────────────────────────────────────────────────────
LAPTOP_USER="rammp"
LAPTOP_IP="10.2.10.4"

JETSON_WS="$HOME/ros2_ws"
LAPTOP_WS="/home/$LAPTOP_USER/ros2_ws"

ROS_SETUP="/opt/ros/humble/setup.zsh"
SESSION="rammp"

# Default launch arguments (mirrors your launch file defaults)
SERIAL_PORT="/dev/ttyACM0"
CHAIR_IP="10.2.10.3"
UE_HOST="127.0.0.1"
LAUNCH_CAMERAS="true"
LAUNCH_ARM="true"
LAUNCH_LUCI="true"
# ───────────────────────────────────────────────────────────────────────────────

# ── Parse CLI flags ────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-cameras)       LAUNCH_CAMERAS="false" ;;
        --no-arm)           LAUNCH_ARM="false" ;;
        --no-luci)          LAUNCH_LUCI="false" ;;
        --serial-port)      SERIAL_PORT="$2"; shift ;;
        --chair-ip)         CHAIR_IP="$2"; shift ;;
        --ue-host)          UE_HOST="$2"; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

# ── Cleanup (runs on Ctrl+C or any exit) ──────────────────────────────────────
cleanup() {
    echo ""
    echo "Shutting down RAMMP system..."

    tmux send-keys -t "$SESSION:jetson" C-c ""
    tmux send-keys -t "$SESSION:laptop" C-c ""
    sleep 3

    # Kill laptop node first (remote)
    ssh -o ConnectTimeout=4 "$LAPTOP_USER@$LAPTOP_IP" \
        "pkill -2 -f 'ros2' || true" 2>/dev/null \
        && echo " Laptop nodes stopped." \
        || echo " (Could not reach laptop — may already be down.)"

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
echo "   Laptop reachable."

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
)
ARGS_STR="${JETSON_ARGS[*]}"

# ── Launch ─────────────────────────────────────────────────────────────────────
echo "Starting RAMMP system..."
echo " Serial: $SERIAL_PORT"
echo " Chair:   $CHAIR_IP"
echo " UE host: $UE_HOST"
echo " Laptop: $LAPTOP_USER@$LAPTOP_IP"
echo ""

# Window 1: GUI — must start first
tmux new-session -d -s "$SESSION" -n "gui" \
    "bash -c 'cd $HOME && export DISPLAY=:1 && ./launch_ui.sh; echo \"[gui] exited.\"; read'"

sleep 2   # give the GUI a moment before launching nodes

# Window 2: Laptop node (SSH)
tmux new-window -t "$SESSION" -n "laptop" \
    "bash -c 'ssh $LAPTOP_USER@$LAPTOP_IP \
        \"source $ROS_SETUP && \
        source ~/.zshrc && \
        conda activate compute && \
        source $LAPTOP_WS/install/setup.zsh && \
        ros2 launch drink_actions_test minimal.launch.py\"; \
    echo \"[laptop] SSH session ended.\"; read'"

# Window 3: Jetson nodes
tmux new-window -t "$SESSION" -n "jetson" \
    "zsh -c 'source $ROS_SETUP && \
            source $JETSON_WS/install/setup.zsh && \
            ros2 launch rammp_prototype_bringup full.launch.py $ARGS_STR | grep '\''system'\''; \
            echo \"[jetson] Launch exited.\"; read'"

# Window 4: Calibration — waits for arm to be ready
tmux new-window -t "$SESSION" -n "calibration" \
    "zsh -c 'source $ROS_SETUP && \
              source $JETSON_WS/install/setup.zsh && \
              echo \"Waiting for arm to be ready...\" && \
              until ros2 node list | grep -q \"/arm_driver_node\"; do sleep 3; done && \
              echo \"Arm ready. Running calibration...\" && \
              ros2 action send_goal /arm/calibrate arm_interfaces/action/Calibrate \"{}\" && \
              echo \"[calibration] done.\"; read'"

# Start on the jetson window
tmux select-window -t "$SESSION:jetson"

echo "Attaching (Ctrl+C to kill everything, Ctrl+B + number to switch windows)..."
tmux attach-session -t "$SESSION"

cleanup
