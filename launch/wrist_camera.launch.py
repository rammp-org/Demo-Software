from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_realsense(context, *args, **kwargs):
    serial_no = LaunchConfiguration("serial_no").perform(context)
    color_width = int(LaunchConfiguration("color_width").perform(context))
    color_height = int(LaunchConfiguration("color_height").perform(context))
    color_fps = float(LaunchConfiguration("color_fps").perform(context))
    depth_width = int(LaunchConfiguration("depth_width").perform(context))
    depth_height = int(LaunchConfiguration("depth_height").perform(context))
    depth_fps = float(LaunchConfiguration("depth_fps").perform(context))
    enable_pointcloud = (
        LaunchConfiguration("enable_pointcloud").perform(context).lower() == "true"
    )
    enable_sync = LaunchConfiguration("enable_sync").perform(context).lower() == "true"

    parameters = [
        {"serial_no": serial_no},
        {"enable_color": True},
        {"rgb_camera.profile": f"{color_width}x{color_height}x{int(color_fps)}"},
        {"enable_depth": True},
        {"depth_module.profile": f"{depth_width}x{depth_height}x{int(depth_fps)}"},
        {"align_depth.enable": True},
        {"pointcloud.enable": enable_pointcloud},
        {"enable_sync": enable_sync},
        {"enable_gyro": True},
        {"enable_accel": True},
        {"unite_imu_method": 2},
        {"hold_back_imu_for_frames": True},
        {"base_frame_id": "wrist_camera_link"},
        {"clip_distance": 2.0},
    ]

    return [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="wrist_camera",
            namespace="wrist_camera",
            parameters=parameters,
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("color_width", default_value="640"),
            DeclareLaunchArgument("color_height", default_value="480"),
            DeclareLaunchArgument("color_fps", default_value="30"),
            DeclareLaunchArgument("depth_width", default_value="640"),
            DeclareLaunchArgument("depth_height", default_value="480"),
            DeclareLaunchArgument("depth_fps", default_value="30"),
            DeclareLaunchArgument("enable_pointcloud", default_value="false"),
            DeclareLaunchArgument("enable_sync", default_value="true"),
            OpaqueFunction(function=_launch_realsense),
        ]
    )
