#!/usr/bin/env bash
# Calibrate the base, then the arm, as soon as each action server is up.
# Lifted from the calibration window of the old launch.sh. Needs the
# workspace sourced; under sheppy the manifest's ros_setup does that.
#
# `set -e` is deliberate: a failed goal exits nonzero, so the sheppy
# `calibration` node shows as crashed instead of sitting in `sleep infinity`.
set -euo pipefail

wait_for_action() {
    echo "Waiting for $1 action server..."
    until ros2 action list 2>/dev/null | grep -q "$1"; do sleep 3; done
}

wait_for_action /base/calibrate
echo "Base ready. Running base calibration..."
sleep 1
ros2 action send_goal /base/calibrate rammp_prototype_interfaces/action/Calibration '{enable: true}'

wait_for_action /arm/calibrate
echo "Arm ready. Running arm calibration..."
ros2 action send_goal /arm/calibrate arm_interfaces/action/Calibrate '{}'
echo "[calibration] done."
