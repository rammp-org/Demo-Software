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
from launch_ros.actions import PushRosNamespace, Node


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

        # Read from config file, default to False
        disable_realsense = str(config.get("disable_realsense", False)).lower()
        disable_orbbec = str(config.get("disable_orbbec", False)).lower()

        # CLI overrides take precedence if explicitly set to "true"
        cli_realsense = LaunchConfiguration("disable_realsense").perform(context)
        cli_orbbec = LaunchConfiguration("disable_orbbec").perform(context)
        if cli_realsense == "true":
            disable_realsense = cli_realsense
        if cli_orbbec == "true":
            disable_orbbec = cli_orbbec

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
                            }.items(),
                        ),
                    ]
                )
            )
            actions.append(
                Node(
                    package='rammp_prototype_utils',
                    executable='image_rotate_node',
                    name='image_rotate_nav',
                    remappings=[
                        ('image_raw',           '/camera/nav/color/image_raw'),
                        ('image_rotated',       '/camera/nav/color/image_rotated'),
                        ('camera_info',         '/camera/nav/color/camera_info'),
                        ('camera_info_rotated', '/camera/nav/color/camera_info_rotated'),
                    ],
                    parameters=[{'rotation_degrees': 90}],
                    output='screen'
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
                description="Set to true to disable orbbec",
            ),
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
            DeclareLaunchArgument(
                "params_file",
                default_value=camera_config,
                description="Path to camera config YAML.",
            ),
            OpaqueFunction(function=load_config),
        ]
    )
