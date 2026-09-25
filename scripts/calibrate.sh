#!/usr/bin/env bash
# Calibrate the base, then the arm, as soon as each action server is up.
# Lifted from the calibration window of the old launch.sh. Needs the
# workspace sourced; under sheppy the manifest's ros_setup does that.
#
# Exits nonzero if either goal does not SUCCEED, so the sheppy `calibration`
# node shows as crashed instead of sitting in `sleep infinity`. That has to
# be checked from the output: `ros2 action send_goal` exits 0 even when the
# goal is rejected or aborted.
set -euo pipefail

wait_for_action() {
    echo "Waiting for $1 action server..."
    until [[ $(ros2 action list 2>/dev/null) == *"$1"* ]]; do sleep 3; done
}

send_goal() {
    local out
    out=$(ros2 action send_goal "$@")
    echo "$out"
    [[ "$out" == *"Goal finished with status: SUCCEEDED"* ]]
}

wait_for_action /base/calibrate
echo "Base ready. Running base calibration..."
sleep 1
send_goal /base/calibrate rammp_prototype_interfaces/action/Calibration '{enable: true}'

wait_for_action /arm/calibrate
echo "Arm ready. Running arm calibration..."
send_goal /arm/calibrate arm_interfaces/action/Calibrate '{}'
echo "[calibration] done."
