#!/bin/bash
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$(id -u)" -eq 0 ]; then
  SUDO=""
else
  SUDO="sudo"
fi
ROSDEP_SOURCE="/etc/ros/rosdep/sources.list.d/50-rammp-custom.list"
ROSDEP_YAML="file://${REPO_ROOT}/hardware/arm_driver/rosdep/python.yaml"
echo "=== Updating apt ==="
$SUDO apt-get update -q
echo "=== Installing Orbbec ROS2 binary ==="
$SUDO apt-get install -y ros-humble-orbbec-camera
echo "=== Registering custom rosdep sources ==="
echo "yaml ${ROSDEP_YAML}" | $SUDO tee "${ROSDEP_SOURCE}"
echo "=== Updating rosdep ==="
rosdep update
echo "=== Installing pip ==="
$SUDO apt-get install -y python3-pip
echo "=== Installing pip dependencies ==="
pip3 install -r "${REPO_ROOT}/hardware/arm_driver/requirements.txt"
echo "=== Installing ROS dependencies ==="
rosdep install \
  --from-paths "${REPO_ROOT}" \
  --ignore-src -r -y \
  --skip-keys "python3-kortex-api python3-pinocchio python3-scipy"
echo "=== Setup complete ==="
