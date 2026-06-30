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

echo "=== Installing gRPC ==="
export MY_INSTALL_DIR="${MY_INSTALL_DIR:-$HOME/.local}"
mkdir -p $MY_INSTALL_DIR
export PATH="$MY_INSTALL_DIR/bin:$PATH"

if [ ! -f "$MY_INSTALL_DIR/lib/libgrpc.a" ]; then
    echo "gRPC not found, building from source..."
    git clone -b v1.56.2 https://github.com/grpc/grpc /tmp/grpc \
      && cd /tmp/grpc \
      && git submodule update --init \
      && mkdir -p cmake/build \
      && cd cmake/build \
      && cmake -DgRPC_INSTALL=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DgRPC_BUILD_TESTS=OFF \
      -DgRPC_PROTOBUF_PROVIDER=module \
      -DCMAKE_INSTALL_PREFIX=$MY_INSTALL_DIR \
      ../.. \
      && make -j$(nproc) \
      && make install \
      && cd $REPO_ROOT
else
    echo "gRPC already installed, skipping build."
fi

echo "=== Registering custom rosdep sources ==="
echo "yaml ${ROSDEP_YAML}" | $SUDO tee "${ROSDEP_SOURCE}"

echo "=== Updating rosdep ==="
rosdep update

echo "=== Installing pip ==="
$SUDO apt-get install -y python3-pip

echo "=== Installing pip dependencies ==="
pip3 install -r "${REPO_ROOT}/hardware/arm_driver/requirements.txt"
pip3 install -r "${REPO_ROOT}/demo_modules/cmu_door_opener/requirements.txt"

echo "=== Installing ROS dependencies ==="
rosdep install \
  --from-paths "${REPO_ROOT}" \
  --ignore-src -r -y \
  --skip-keys "python3-kortex-api python3-pinocchio python3-scipy"


echo "=== Installing Orbbec udev rules ==="
# udev rules allow non-root users to communicate with Orbbec cameras over USB.
# Without these rules, the camera node fails with a USB permission error
# even if the device is detected by lsusb.
# The rules file location depends on how Orbbec was installed:
#   - From ROS apt package:    /opt/ros/humble/share/orbbec_camera/scripts/
#   - From ROS source build:   ${REPO_ROOT}/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts/
#   - From pyorbbecsdk:        ${HOME}/pyorbbecsdk/scripts/env_setup/
ORBBEC_UDEV_RULES=""
SEARCH_PATHS=(
    "/opt/ros/humble/share/orbbec_camera/scripts/99-obsensor-libusb.rules"
    "/opt/ros/humble/lib/orbbec_camera/orbbec_camera/scripts/99-obsensor-libusb.rules"
    "${REPO_ROOT}/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules"
    "${HOME}/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules"
    "${HOME}/pyorbbecsdk/scripts/env_setup/99-obsensor-libusb.rules"
)
for path in "${SEARCH_PATHS[@]}"; do
    if [ -f "${path}" ]; then
        ORBBEC_UDEV_RULES="${path}"
        break
    fi
done
if [ -n "${ORBBEC_UDEV_RULES}" ]; then
    $SUDO cp "${ORBBEC_UDEV_RULES}" /etc/udev/rules.d/
    $SUDO udevadm control --reload-rules
    $SUDO udevadm trigger
    echo "Orbbec udev rules installed from: ${ORBBEC_UDEV_RULES}"
elif [ -f "/etc/udev/rules.d/99-obsensor-libusb.rules" ]; then
    echo "Orbbec udev rules already installed at /etc/udev/rules.d/ — skipping."
else
    echo "WARNING: Orbbec udev rules not found in any known location."
    echo "Install ros-humble-orbbec-camera or pyorbbecsdk first,"
    echo "then re-run setup.sh to install udev rules."
fi

echo "=== Installing MEBot keypad udev rules ==="
# Stable /dev/mebot_keypad symlink for the SayoDevice 1x4P keypad (keyboard_driver).
# Points at the one event node (USB interface 0) that emits the W/E/R/T presses;
# the sibling nodes advertise the keys but never fire.
KEYPAD_UDEV_RULES="${REPO_ROOT}/hardware/keyboard_driver/udev/99-mebot-keypad.rules"
if [ -f "${KEYPAD_UDEV_RULES}" ]; then
    $SUDO cp "${KEYPAD_UDEV_RULES}" /etc/udev/rules.d/
    $SUDO udevadm control --reload-rules
    $SUDO udevadm trigger
    echo "MEBot keypad udev rules installed from: ${KEYPAD_UDEV_RULES}"
else
    echo "WARNING: MEBot keypad udev rules not found at ${KEYPAD_UDEV_RULES}."
fi

echo "=== Configuring Jetson max performance mode ==="
bash "${REPO_ROOT}/scripts/jetson_max_performance.sh" "$@"

echo "=== Setup complete ==="
