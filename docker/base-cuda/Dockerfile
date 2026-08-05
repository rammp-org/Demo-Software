# Shared CUDA base image for RAMMP module containers that need the GPU.
#
# Build once (and after any change to THIS file):
#   docker build -t rammp-base-cuda:humble docker/base-cuda      (or: make base)
#
# This is the GPU sibling of docker/base/Dockerfile. Use it only for modules
# that actually import torch / run CUDA kernels; everything else should stay on
# the much smaller rammp-base:humble.
#
# Why a separate base rather than `runtime: nvidia` on the slim one: on this
# Jetson the nvidia container runtime injects only the DRIVER libraries and
# device nodes (see /etc/nvidia-container-runtime/host-files-for-container.d/
# drivers.csv + devices.csv). It does NOT inject the CUDA toolkit runtime --
# libcudart / libcublas live under /usr/local/cuda on the host and must be
# present IN the image. l4t-jetpack ships them, ros:humble does not, which is
# why a CUDA torch wheel on the slim base dies with
# "OSError: libcudart.so.12: cannot open shared object file".
#
# Services built on this image must also set `runtime: nvidia` in
# docker-compose.yml so the driver half gets injected at run time.

FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

WORKDIR /ros2_ws

# Ubuntu 22.04 (jammy) + Python 3.10 here, same as ros:humble, so the ROS 2
# Humble apt packages and the JetPack 6 / cu126 wheels agree on ABI.
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# ROS 2 apt repository. l4t-jetpack has no ROS at all, so unlike the slim base
# we install the distro ourselves.
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release software-properties-common && \
    add-apt-repository -y universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    rm -rf /var/lib/apt/lists/*

# ROS Humble + the middleware and build tooling that ros:humble would have given
# us for free. Kept in one layer so the apt cache is dropped once.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-rmw-cyclonedds-cpp \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-pip \
      build-essential \
      git && \
    rm -rf /var/lib/apt/lists/*

# Warm the rosdep cache once, so each module's `rosdep install` skips the slow
# `rosdep update` step. rosdep is not pre-initialized here (ros:humble does that
# in its own build), so init first.
RUN rosdep init && rosdep update --rosdistro humble

# Jammy ships pip 22.0.2, which fails on `name @ git+https://...#subdirectory=`
# requirements with "metadata has 'unknown'". Upgrade once here so no module on
# this base has to.
RUN pip install --no-cache-dir -U pip

# Entrypoint: identical to the slim base -- always source ROS, and source the
# module workspace too if one has been built on top of this image. Built inline
# so there is no external script file to manage or get CRLF-corrupted.
RUN printf '%s\n' \
      '#!/bin/bash' \
      'set -e' \
      'source /opt/ros/humble/setup.bash' \
      'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' \
      'exec "$@"' \
      > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
