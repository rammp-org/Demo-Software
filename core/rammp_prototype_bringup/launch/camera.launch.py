import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wrist_camera_serial",
                default_value="",
                description="Serial number of the RealSense D435i wrist camera.",
            ),
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
                }.items(),
            ),
        ]
    )
