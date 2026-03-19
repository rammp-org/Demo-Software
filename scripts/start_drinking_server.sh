#!/bin/bash
# Run this on the Jetson to start the dummy drinking action servers.
#
# Prerequisites:
#   1. Build the workspace:
#      colcon build --packages-select cornell_feeding_interfaces cornell_feeding
#   2. Source the workspace:
#      source install/setup.bash
#
# Usage:
#   ./scripts/start_drinking_server.sh
#   ./scripts/start_drinking_server.sh 42          # custom ROS_DOMAIN_ID

set -e

export ROS_DOMAIN_ID=${1:-0}
echo "Starting drinking_node server with ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

ros2 run cornell_feeding drinking_node
