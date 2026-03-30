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

    # Default config. Override at launch time with:
    #   params_file:=$(ros2 pkg prefix rammp_prototype_bringup --share)/config/<variant>.yaml
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
                description="Set to true to disable realsense",
            ),
            DeclareLaunchArgument(
                "disable_orbbec",
                default_value="false",
                description="Set to true to disable orbbec",
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
            # ── Config file override ───────────────────────────────────────
            DeclareLaunchArgument(
                "params_file",
                default_value=camera_config,
                description=(
                    "Absolute path to the camera config YAML. "
                    "Defaults to camera_config.yaml."
                ),
            ),
            # ── Wrist camera (RealSense D435i) ────────────────────────────
            # Delayed 8 s to allow the Orbbec USB stack to settle first.
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
                            # params_file is a launch arg in rs_launch.py,
                            # not a node parameter — must be in launch_arguments.
                            "params_file": LaunchConfiguration("params_file"),
                        }.items(),
                        condition=UnlessCondition(
                            LaunchConfiguration("disable_realsense")
                        ),
                    )
                ],
            ),
            # ── Nav camera (Orbbec Gemini 336L) ───────────────────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(orbbec_launch),
                launch_arguments={
                    "camera_name": "nav",
                    "camera_namespace": "camera",
                    "serial_number": LaunchConfiguration("nav_camera_serial"),
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
                    # params_file is a launch arg in gemini_330_series.launch.py,
                    # not a node parameter — must be in launch_arguments.
                    "params_file": LaunchConfiguration("params_file"),
                }.items(),
                condition=UnlessCondition(LaunchConfiguration("disable_orbbec")),
            ),
        ]
    )
