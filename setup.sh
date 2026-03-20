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

echo "=== Installing gRPC and protobuf ==="
export MY_INSTALL_DIR=$HOME/.local
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
      && cd $REPO_ROOT \
      && $SUDO apt-get install -y libprotobuf-dev
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

echo "=== Installing ROS dependencies ==="
rosdep install \
  --from-paths "${REPO_ROOT}" \
  --ignore-src -r -y \
  --skip-keys "python3-kortex-api python3-pinocchio python3-scipy"

echo "=== Setup complete ==="
