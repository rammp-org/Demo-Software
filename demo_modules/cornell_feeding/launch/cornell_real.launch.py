"""
Real-robot bring-up for the cornell_feeding drink stack on the physical Kinova Gen3.

Composes the pieces the drink_action_server needs in run_on_robot mode by INCLUDING
existing core launches (rather than recreating any TF/URDF/camera node):

  - rammp_prototype_description/description.launch.py : robot_state_publisher (Gen3 +
    Robotiq URDF -> base_link..end_effector_link) and the end_effector_link -> wrist
    camera static TF. We pass camera_frame:=wrist_camera_link so that mount frame matches
    the RealSense base_frame_id, completing base_link -> wrist_color_optical_frame.
  - arm_driver : the /arm/* hardware backend that owns the Kortex link to the Gen3
    (reused verbatim from rammp_prototype_bringup/full.launch.py).
  - rammp_prototype_bringup/camera.launch.py : the wrist RealSense (640x480, aligned
    depth, initial_reset) published under /camera/wrist/*, with the nav cameras disabled.
  - cornell_feeding/cornell_feeding.launch.py : the drink_action_server (run_on_robot)
    plus the map->world static TF.

This intentionally does NOT call /arm/set_mode. The arm only accepts cornell commands in
ORDER_DRINK / DRINKING, which the orchestrator (rammp_prototype_behavior/system_control)
sets on state entry -- the same pattern cmu_door_opener follows. For bench testing without
the orchestrator, set it manually once the arm is connected:

    ros2 service call /arm/set_mode arm_interfaces/srv/SetMode "{mode: 2}"   # ORDER_DRINK

Real mode also requires the one-time head calibration (mediapipe_config/drink/*.npy via
`python3 -m rammp.perception.head_perception.calibrate_head --tool drink`); without it the
node fails at startup with a clear error.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scene_config = LaunchConfiguration("scene_config")

    description_launch = os.path.join(
        get_package_share_directory("rammp_prototype_description"),
        "launch",
        "description.launch.py",
    )
    camera_launch = os.path.join(
        get_package_share_directory("rammp_prototype_bringup"),
        "launch",
        "camera.launch.py",
    )
    camera_params = os.path.join(
        get_package_share_directory("rammp_prototype_bringup"),
        "config",
        "camera_wrist.yaml",
    )
    cornell_launch = os.path.join(
        get_package_share_directory("cornell_feeding"),
        "launch",
        "cornell_feeding.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value="wheelchair"),
        # TF tree + end_effector_link -> wrist_camera_link mount extrinsic. The
        # camera_frame override makes the mount-TF child equal the RealSense root
        # frame so base_link -> wrist_color_optical_frame resolves for perception.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={"camera_frame": "wrist_camera_link"}.items(),
        ),
        # Kinova Gen3 hardware backend (owns the Kortex link; provides /arm/*).
        Node(
            package="arm_driver",
            executable="arm_driver",
            name="arm_driver_node",
            output="screen",
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
        ),
        # Wrist RealSense (D435i), published under /camera/wrist/*.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                "params_file": camera_params,
                "disable_nav1": "true",
                "disable_nav2": "true",
            }.items(),
        ),
        # The drink_action_server itself, in real mode.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cornell_launch),
            launch_arguments={
                "run_on_robot": "true",
                "scene_config": scene_config,
            }.items(),
        ),
    ])
