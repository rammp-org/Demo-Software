# Shared base image for all RAMMP module containers.
#
# Build once (and after any change to THIS file):
#   docker build -t rammp-base:humble docker/base
#
# Every module Dockerfile then starts with `FROM rammp-base:humble` and only
# adds its own source, dependencies, and build step.

FROM ros:humble

WORKDIR /ros2_ws

# The whole system's middleware (Cyclone DDS) + pip for Python dependencies.
# Baked in here so no module image has to install them again.
RUN apt-get update && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Warm the rosdep cache once, so each module's `rosdep install` skips the slow
# `rosdep update` step.
RUN rosdep update --rosdistro humble

# Entrypoint: always source ROS, and source the module workspace too if one has
# been built on top of this image. Built inline so there is no external script
# file to manage or get CRLF-corrupted.
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
