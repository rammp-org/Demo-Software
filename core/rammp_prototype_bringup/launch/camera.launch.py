import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


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
        "camera_demo_main.yaml",
    )

    def load_config(context, *args, **kwargs):
        params_file = LaunchConfiguration("params_file").perform(context)
        config = {}
        if params_file and os.path.exists(params_file):
            with open(params_file, "r") as f:
                config = yaml.safe_load(f) or {}

        disable_realsense = str(config.get("disable_realsense", False)).lower()
        disable_orbbec = str(config.get("disable_orbbec", False)).lower()
        disable_nav2 = str(config.get("disable_nav2", False)).lower()

        # Allow CLI overrides to take precedence
        cli_realsense = LaunchConfiguration("disable_realsense").perform(context)
        cli_orbbec = LaunchConfiguration("disable_orbbec").perform(context)
        cli_nav2 = LaunchConfiguration("disable_nav2").perform(context)
        if cli_realsense != "false":
            disable_realsense = cli_realsense
        if cli_orbbec != "false":
            disable_orbbec = cli_orbbec
        if cli_nav2 != "false":
            disable_nav2 = cli_nav2

        actions = []

        if disable_realsense != "true":
            actions.append(
                TimerAction(
                    period=8.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(realsense_launch),
                            launch_arguments={
                                "camera_name": "wrist",
                                "serial_no": LaunchConfiguration("wrist_camera_serial"),
                                "base_frame_id": "wrist_camera_link",
                                "rgb_camera.profile": "640x480x30",
                                "depth_module.profile": "640x480x30",
                                "align_depth.enable": "true",
                                "enable_gyro": "true",
                                "enable_accel": "true",
                                "unite_imu_method": "2",
                                "pointcloud.enable": "false",
                                "log_level": "warn",
                                "params_file": params_file,
                            }.items(),
                        )
                    ],
                )
            )

        if disable_orbbec != "true":
            actions.append(
                GroupAction(
                    actions=[
                        PushRosNamespace("camera"),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(orbbec_launch),
                            launch_arguments={
                                "camera_name": "nav",
                                "serial_number": LaunchConfiguration(
                                    "nav_camera_serial"
                                ),
                                "base_frame_id": "nav_camera_link",
                                "enable_point_cloud": "true",
                                "enable_hole_filling_filter": "false",
                                "hole_filling_filter_mode": "NEAREST_NEIGHBOR_MAX",
                                "enable_spatial_filter": "true",
                                "spatial_filter_magnitude": "1",
                                "spatial_filter_alpha": "0.5",
                                "spatial_filter_diff_threshold": "25",
                                "enable_temporal_filter": "true",
                                "temporal_filter_diff_threshold": "0.3",
                                "temporal_filter_weight": "0.4",
                                "enable_noise_removal_filter": "true",
                                "noise_removal_filter_min_diff": "128",
                                "noise_removal_filter_max_size": "100",
                                "enable_threshold_filter": "false",
                                "threshold_filter_min": "100",
                                "threshold_filter_max": "10000",
                                "enable_hdr_merge": "true",
                                "depth_width": "640",
                                "depth_height": "400",
                                "depth_fps": "30",
                                "depth_registration": "true",
                                "color_width": "640",
                                "color_height": "400",
                                "enable_accel": "true",
                                "enable_gyro": "true",
                                "color_fps": "30",
                                "exposure_range_mode": "ultimate",
                                "laser_energy_level": "4",
                                "enable_ir_auto_exposure": "true",
                                "config_file_path": params_file,
                            }.items(),
                        ),
                    ]
                )
            )

        if disable_nav2 != "true":
            actions.append(
                GroupAction(
                    actions=[
                        PushRosNamespace("camera"),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(orbbec_launch),
                            launch_arguments={
                                "camera_name": "nav2",
                                "serial_number": LaunchConfiguration(
                                    "nav2_camera_serial"
                                ),
                                "base_frame_id": "nav2_camera_link",
                                "enable_point_cloud": "true",
                                "enable_hole_filling_filter": "false",
                                "hole_filling_filter_mode": "NEAREST_NEIGHBOR_MAX",
                                "enable_spatial_filter": "true",
                                "spatial_filter_magnitude": "1",
                                "spatial_filter_alpha": "0.5",
                                "spatial_filter_diff_threshold": "25",
                                "enable_temporal_filter": "true",
                                "temporal_filter_diff_threshold": "0.3",
                                "temporal_filter_weight": "0.4",
                                "enable_noise_removal_filter": "true",
                                "noise_removal_filter_min_diff": "128",
                                "noise_removal_filter_max_size": "100",
                                "enable_threshold_filter": "false",
                                "threshold_filter_min": "100",
                                "threshold_filter_max": "10000",
                                "enable_hdr_merge": "true",
                                "depth_width": "640",
                                "depth_height": "400",
                                "depth_fps": "30",
                                "depth_registration": "true",
                                "color_width": "640",
                                "color_height": "400",
                                "enable_accel": "true",
                                "enable_gyro": "true",
                                "color_fps": "30",
                                "exposure_range_mode": "ultimate",
                                "laser_energy_level": "4",
                                "enable_ir_auto_exposure": "true",
                                "config_file_path": params_file,
                            }.items(),
                        ),
                    ]
                )
            )

        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "disable_realsense",
                default_value="false",
                description="Set to true to disable realsense",
            ),
            DeclareLaunchArgument(
                "disable_orbbec",
                default_value="false",
                description="Set to true to disable orbbec nav camera",
            ),
            DeclareLaunchArgument(
                "disable_nav2",
                default_value="false",
                description="Set to true to disable orbbec nav2 shoulder camera",
            ),
            # ── Wrist camera serial (RealSense D435i) ─────────────────────
            DeclareLaunchArgument(
                "wrist_camera_serial",
                default_value="",
                description="Serial number of the RealSense D435i wrist camera.",
            ),
            # ── Nav camera serial (Orbbec Gemini 336L) ────────────────────
            DeclareLaunchArgument(
                "nav_camera_serial",
                default_value="",
                description="Serial number of the Orbbec Gemini 336L navigation camera.",
            ),
            # ── Nav2 camera serial (Orbbec Gemini 336L) ───────────────────
            DeclareLaunchArgument(
                "nav2_camera_serial",
                default_value="",
                description="Serial number of the Orbbec Gemini 336L shoulder camera.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=camera_config,
                description="Path to camera config YAML.",
            ),
            OpaqueFunction(function=load_config),
        ]
    )
