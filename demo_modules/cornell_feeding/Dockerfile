# cornell_feeding, built on the shared base image.
#
# Build the base image first:  make base
# Build context is the REPO ROOT (see docker-compose.yml).
#
# This image is what replaces setup.sh for this module: requirements.txt owns
# every Python dependency, so nothing lands in the host's site-packages and no
# custom rosdep source has to be registered under /etc.
#
# Scope is the module only - the drink_action_server plus its map->world static
# TF. arm_driver (and with it kortex_api and the protobuf conflict), the wrist
# RealSense, and robot_state_publisher stay on the host and reach this container
# over host-network DDS.

FROM rammp-base:humble

# The only interface package this module imports. The vendored rammp stack uses
# cornell_feeding_interfaces exclusively - arm_client drives arm_driver over
# std_srvs/sensor_msgs/geometry_msgs, not arm_interfaces - so copying just this
# one avoids building five unrelated rosidl packages on every build.
COPY interfaces/cornell_feeding_interfaces/ /ros2_ws/src/cornell_feeding_interfaces/

# ROS/apt deps only (cached unless a package.xml changes).
#
# Every python3-*-pip key is skipped so requirements.txt below is the single
# source of truth for the Python stack:
#   - The PRPL keys and python3-protobuf-pip resolve on the dev Jetson ONLY
#     because setup.sh registers /etc/ros/rosdep/sources.list.d/50-rammp-custom.list.
#     A clean container has no such source, so rosdep would hard-fail on them.
#   - python3-mediapipe-pip / -open3d-pip / -pybullet-pip DO resolve upstream,
#     but would pip-install unpinned versions behind requirements.txt's back.
#   - python3-scipy is skipped because pybullet_helpers pins scipy==1.14.0 and
#     the apt build would fight it.
COPY demo_modules/cornell_feeding/package.xml /ros2_ws/src/cornell_feeding/package.xml
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y \
      --skip-keys "python3-scipy python3-mediapipe-pip python3-open3d-pip \
python3-pybullet-pip python3-protobuf-pip python3-pybullet-helpers-pip \
python3-relational-structs-pip python3-prpl-utils-pip python3-tomsutils-pip" && \
    rm -rf /var/lib/apt/lists/*

# rammp-base ships pip 22.0.2, which fails on `name @ git+https://...
# #subdirectory=` requirements with "metadata has 'unknown'". All four PRPL
# packages are that form, so pip must be upgraded before they are installed.
RUN pip install --no-cache-dir -U pip

# Python deps (mediapipe / open3d / pybullet-arm64 / PRPL stack / protobuf<5)
# - cached unless requirements.txt changes.
COPY demo_modules/cornell_feeding/requirements.txt /ros2_ws/src/cornell_feeding/requirements.txt
RUN pip install --no-cache-dir -r /ros2_ws/src/cornell_feeding/requirements.txt

# Source + build (the only layers that re-run when you edit the node).
COPY demo_modules/cornell_feeding/ /ros2_ws/src/cornell_feeding/
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

CMD ["ros2", "launch", "cornell_feeding", "cornell_feeding.launch.py"]
