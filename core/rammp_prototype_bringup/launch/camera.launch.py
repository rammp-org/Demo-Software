import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"),
        "launch",
        "gemini_330_series.launch.py",
    )

    camera_config = os.path.join(
        get_package_share_directory("rammp_prototype_bringup"),
        "config",
        "camera_config.yaml",
    )

    return LaunchDescription(
        [
            # ── Enable / disable flags ─────────────────────────────────────
            DeclareLaunchArgument(
                "disable_realsense",
                default_value="false",
                description="Set to true to disable the wrist RealSense camera.",
            ),
            DeclareLaunchArgument(
                "disable_orbbec",
                default_value="false",
                description="Set to true to disable the nav Orbbec camera.",
            ),
            # ── Serial number overrides ────────────────────────────────────
            DeclareLaunchArgument(
                "wrist_camera_serial",
                default_value="",
                description="Serial number of the RealSense D435i wrist camera.",
            ),
            DeclareLaunchArgument(
                "nav_camera_serial",
                default_value="",
                description="Serial number of the Orbbec Gemini 336L navigation camera.",
            ),
            # ── Wrist camera (RealSense D435i) ─────────────────────────────
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(realsense_launch),
                        launch_arguments={
                            "serial_no": LaunchConfiguration("wrist_camera_serial"),
                            "params_file": camera_config,
                            "log_level": "warn",
                        }.items(),
                        condition=UnlessCondition(
                            LaunchConfiguration("disable_realsense")
                        ),
                    )
                ],
            ),
            # ── Nav camera (Orbbec Gemini 336L) ────────────────────────────
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(orbbec_launch),
                        launch_arguments={
                            "serial_number": LaunchConfiguration("nav_camera_serial"),
                            "params_file": camera_config,
                        }.items(),
                        condition=UnlessCondition(
                            LaunchConfiguration("disable_orbbec")
                        ),
                    )
                ],
            ),
        ]
    )
