#!/bin/bash
# Run this on the laptop to call the dummy drinking action servers on the Jetson.
#
# Prerequisites:
#   1. cornell_feeding_interfaces must be built on this machine too:
#      colcon build --packages-select cornell_feeding_interfaces
#   2. Source the workspace:
#      source install/setup.bash
#   3. Both machines must be on the same network.
#   4. Use the same ROS_DOMAIN_ID as the Jetson.
#
# Usage:
#   ./scripts/start_feeding_client.sh
#   ./scripts/start_feeding_client.sh 42           # custom ROS_DOMAIN_ID

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_RUN="$SCRIPT_DIR/../../src/rammp/integration/demo_run.py"

export ROS_DOMAIN_ID=${1:-0}
echo "Running feeding demo client with ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

python3 "$DEMO_RUN" "$@"
