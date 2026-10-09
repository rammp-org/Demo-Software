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
    # ── Launch file paths ─────────────────────────────────────────────────
    # Locate the RealSense ROS2 wrapper launch file installed via apt
    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    # Locate the Orbbec ROS2 wrapper launch file installed via apt
    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"),
        "launch",
        "gemini_330_series.launch.py",
    )

    # Default config file — both cameras enabled, loaded unless overridden
    # by the params_file launch argument
    camera_config = os.path.join(
        get_package_share_directory("rammp_prototype_bringup"),
        "config",
        "camera_demo_main.yaml",
    )

    def load_config(context, *args, **kwargs):
        # OpaqueFunction runs at launch time (not parse time), so we can
        # read the YAML file and use its values to decide which cameras to launch
        params_file = LaunchConfiguration("params_file").perform(context)
        config = {}
        if params_file and os.path.exists(params_file):
            with open(params_file, "r") as f:
                config = yaml.safe_load(f) or {}

        # Read disable flags from YAML config, defaulting to False (enabled)
        disable_realsense = str(config.get("disable_realsense", False)).lower()
        disable_nav1 = str(config.get("disable_nav1", False)).lower()
        disable_nav2 = str(config.get("disable_nav2", False)).lower()

        # CLI overrides take precedence ONLY if explicitly set to "true".
        # We check for == "true" (not != "false") to avoid the default "false"
        # from DeclareLaunchArgument silently overriding a "true" in the YAML.
        cli_realsense = LaunchConfiguration("disable_realsense").perform(context)
        cli_nav1 = LaunchConfiguration("disable_nav1").perform(context)
        cli_nav2 = LaunchConfiguration("disable_nav2").perform(context)
        if cli_realsense == "true":
            disable_realsense = cli_realsense
        if cli_nav1 == "true":
            disable_nav1 = cli_nav1
        if cli_nav2 == "true":
            disable_nav2 = cli_nav2

        # Read per-camera parameters from YAML ROS parameter sections
        wrist_params = config.get("/wrist", {}).get("ros__parameters", {})
        nav1_params = config.get("/camera/nav1", {}).get("ros__parameters", {})
        nav2_params = config.get("/camera/nav2", {}).get("ros__parameters", {})

        # camera_name and base_frame_id — YAML with hardcoded fallbacks
        wrist_camera_name = wrist_params.get("camera_name", "wrist")
        wrist_base_frame = wrist_params.get("base_frame_id", "wrist_camera_link")
        nav1_camera_name = nav1_params.get("camera_name", "nav1")
        nav1_base_frame = nav1_params.get("base_frame_id", "nav1_camera_link")
        nav2_camera_name = nav2_params.get("camera_name", "nav2")
        nav2_base_frame = nav2_params.get("base_frame_id", "nav2_camera_link")

        # Serial numbers — YAML default, CLI override if non-empty
        wrist_serial = wrist_params.get("serial_no", "")
        nav1_serial = nav1_params.get("serial_number", "")
        nav2_serial = nav2_params.get("serial_number", "")

        cli_wrist_serial = LaunchConfiguration("wrist_camera_serial").perform(context)
        cli_nav1_serial = LaunchConfiguration("nav1_camera_serial").perform(context)
        cli_nav2_serial = LaunchConfiguration("nav2_camera_serial").perform(context)

        if cli_wrist_serial:
            wrist_serial = cli_wrist_serial
        if cli_nav1_serial:
            nav1_serial = cli_nav1_serial
        if cli_nav2_serial:
            nav2_serial = cli_nav2_serial

        # Remove our custom args from the launch context so they don't leak
        # into sub-launch files (RealSense, Orbbec) and cause "unsupported
        # parameter" warnings. All values have already been extracted above.
        for key in [
            "params_file",
            "disable_realsense",
            "disable_nav1",
            "disable_nav2",
            "wrist_camera_serial",
            "nav1_camera_serial",
            "nav2_camera_serial",
        ]:
            context.launch_configurations.pop(key, None)

        actions = []

        # ── Wrist camera (RealSense D435i) ────────────────────────────────
        # Delayed by 8 seconds to give the Orbbec cameras time to initialize
        # first. RealSense requires extra startup time on the Jetson.
        # Note: The params_file is NOT passed to RealSense to avoid warnings
        # about unrecognized keys (disable_nav1, nav1_camera_serial, etc.).
        # These keys are only used by OpaqueFunction above.
        if disable_realsense != "true":
            actions.append(
                TimerAction(
                    period=8.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(realsense_launch),
                            launch_arguments={
                                "camera_name": wrist_camera_name,
                                "serial_no": wrist_serial,
                                "base_frame_id": wrist_base_frame,
                                "rgb_camera.color_profile": "640x480x15",
                                "depth_module.depth_profile": "640x480x15",
                                "align_depth.enable": "true",
                                "enable_gyro": "true",
                                "enable_accel": "true",
                                # unite_imu_method=2: linear interpolation merges
                                # gyro + accel into a single /wrist/imu topic
                                "unite_imu_method": "2",
                                "pointcloud.enable": "false",
                                "log_level": "warn",
                            }.items(),
                        )
                    ],
                )
            )
            actions.append(
                Node(
                    package="image_transport",
                    executable="republish",
                    name="wrist_color_republish",
                    arguments=["raw", "compressed"],
                    remappings=[
                        ("in", "/camera/wrist/color/image_raw"),
                        (
                            "out/compressed",
                            "/camera/wrist/color/image_raw/compressed_png",
                        ),
                    ],
                    parameters=[{"out.format": "png", "out.png_level": 6}],
                    output="screen",
                )
            )

            # Depth
            actions.append(
                Node(
                    package="image_transport",
                    executable="republish",
                    name="wrist_depth_republish",
                    arguments=["raw", "compressed"],
                    remappings=[
                        ("in", "/camera/wrist/aligned_depth_to_color/image_raw"),
                        (
                            "out/compressed",
                            "/camera/wrist/aligned_depth_to_color/image_raw/compressed_png",
                        ),
                    ],
                    parameters=[{"out.format": "png", "out.png_level": 6}],
                    output="screen",
                )
            )

        # ── Nav1 camera (Orbbec Gemini 336L) ──────────────────────────────
        # PushRosNamespace("camera") prepends /camera to all topics published
        # by this node, giving us /camera/nav1/color/image_raw etc.
        # Serial number is read from the YAML config (CLI arg overrides if set).
        # It must be passed as a launch arg — Orbbec's config loader explicitly
        # skips serial_number when reading config_file_path.
        if disable_nav1 != "true":
            actions.append(
                GroupAction(
                    actions=[
                        PushRosNamespace("camera"),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(orbbec_launch),
                            launch_arguments={
                                "camera_name": nav1_camera_name,
                                "serial_number": nav1_serial,
                                "base_frame_id": nav1_base_frame,
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
                                "depth_fps": "10",
                                "depth_registration": "true",
                                "color_width": "640",
                                "color_height": "400",
                                "enable_accel": "true",
                                "enable_gyro": "true",
                                "color_fps": "10",
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
                    package="rammp_prototype_utils",
                    executable="image_rotate_node",
                    name="image_rotate_color_nav1",
                    remappings=[
                        ("image_raw", "/camera/nav1/color/image_raw"),
                        ("image_rotated", "/camera/nav1/color/image_rotated"),
                        ("camera_info", "/camera/nav1/color/camera_info"),
                        (
                            "camera_info_rotated",
                            "/camera/nav1/color/camera_info_rotated",
                        ),
                    ],
                    parameters=[{"rotation_degrees": 90}],
                    output="screen",
                )
            )
            actions.append(
                Node(
                    package="rammp_prototype_utils",
                    executable="image_rotate_node",
                    name="image_rotate_depth_nav1",
                    remappings=[
                        ("image_raw", "/camera/nav1/depth/image_raw"),
                        ("image_rotated", "/camera/nav1/depth/image_rotated"),
                        ("camera_info", "/camera/nav1/depth/camera_info"),
                        (
                            "camera_info_rotated",
                            "/camera/nav1/depth/camera_info_rotated",
                        ),
                    ],
                    parameters=[{"rotation_degrees": 90}],
                    output="screen",
                )
            )

        # ── Nav2 shoulder camera (Orbbec Gemini 336L) ─────────────────────
        # Second Orbbec camera mounted on the shoulder of the robot.
        # Identical configuration to nav1 camera — serial number differentiates
        # the two devices at the USB driver level.
        # Topics publish under /camera/nav2/...
        if disable_nav2 != "true":
            actions.append(
                GroupAction(
                    actions=[
                        PushRosNamespace("camera"),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(orbbec_launch),
                            launch_arguments={
                                "camera_name": nav2_camera_name,
                                "serial_number": nav2_serial,
                                "base_frame_id": nav2_base_frame,
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
                                "depth_fps": "10",
                                "depth_registration": "true",
                                "color_width": "640",
                                "color_height": "400",
                                "enable_accel": "true",
                                "enable_gyro": "true",
                                "color_fps": "10",
                                "exposure_range_mode": "ultimate",
                                "laser_energy_level": "4",
                                "enable_ir_auto_exposure": "true",
                            }.items(),
                        ),
                    ]
                )
            )

        return actions

    return LaunchDescription(
        [
            # ── Disable flags — can be set via CLI or YAML config file ─────
            DeclareLaunchArgument(
                "disable_realsense",
                default_value="false",
                description="Set to true to disable realsense",
            ),
            DeclareLaunchArgument(
                "disable_nav1",
                default_value="false",
                description="Set to true to disable orbbec nav1 camera",
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
            # ── Nav1 camera serial (Orbbec Gemini 336L) ───────────────────
            DeclareLaunchArgument(
                "nav1_camera_serial",
                default_value="",
                description="Serial number of the Orbbec Gemini 336L nav1 camera.",
            ),
            # ── Nav2 camera serial (Orbbec Gemini 336L) ───────────────────
            DeclareLaunchArgument(
                "nav2_camera_serial",
                default_value="",
                description="Serial number of the Orbbec Gemini 336L nav2 shoulder camera.",
            ),
            # ── Config file — defaults to camera_demo_main.yaml ───────────
            # Override with params_file:=<path> to use a different config.
            # Note: The top-level keys in this file (disable_realsense,
            # disable_nav1, disable_nav2) are read by OpaqueFunction only and
            # are not passed to camera nodes, avoiding unsupported parameter warnings.
            DeclareLaunchArgument(
                "params_file",
                default_value=camera_config,
                description="Path to camera config YAML.",
            ),
            # OpaqueFunction reads the YAML and returns the correct actions
            # based on which cameras are enabled
            OpaqueFunction(function=load_config),
        ]
    )
