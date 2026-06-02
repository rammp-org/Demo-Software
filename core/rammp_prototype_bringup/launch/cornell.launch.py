import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("rammp_prototype_bringup")

    # ── Launch arguments ──────────────────────────────────────────────────────

    # Per-node enable flags
    launch_description_arg = DeclareLaunchArgument(
        "launch_description",
        default_value="true",
        description="Launch robot_state_publisher and wrist camera static TF",
    )
    launch_arm_driver_arg = DeclareLaunchArgument(
        "launch_arm_driver",
        default_value="true",
        description="Launch Kinova arm driver node",
    )
    launch_cameras_arg = DeclareLaunchArgument(
        "launch_cameras",
        default_value="true",
        description="Launch camera nodes (RealSense wrist + Orbbec nav)",
    )
    # ── Core infrastructure ──────────────────────────────────────+─────────────

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rammp_prototype_description"),
                "launch",
                "description.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_description")),
    )

    arm_driver_node = Node(
        package="arm_driver",
        executable="arm_driver",
        name="arm_driver_node",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("launch_arm_driver")),
    )

    # ── Demo modules ──────────────────────────────────────────────────────────

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "camera.launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(
                get_package_share_directory("rammp_prototype_bringup"),
                "config",
                "camera_demo_main.yaml",
            )
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_cameras")),
    )

    return LaunchDescription(
        [
            # Arguments — enable/disable flags
            launch_description_arg,
            launch_arm_driver_arg,
            launch_cameras_arg,
            # Core infrastructure
            description_launch,
            arm_driver_node,
            # Demo modules
            cameras_launch,
        ]
    )
