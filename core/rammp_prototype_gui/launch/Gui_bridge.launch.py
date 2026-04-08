from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ue_host",
                default_value="127.0.0.1",
                description="IP address of the UE host",
            ),
            DeclareLaunchArgument(
                "use_shared_memory",
                default_value="false",
                description="Whether to use shared memory for communication with UE",
            ),
            DeclareLaunchArgument(
                "ue_preset",
                default_value="RCPS",
                description="UE remote control preset name",
            ),
            DeclareLaunchArgument(
                "wrist_camera_namespace",
                default_value="/camera/wrist",
                description="Namespace for the wrist camera topics",
            ),
            DeclareLaunchArgument(
                "nav_camera_namespace_1",
                default_value="/camera/nav1",
                description="Namespace for the first navigation camera topics",
            ),
            DeclareLaunchArgument(
                "nav_camera_namespace_2",
                default_value="/camera/nav2",
                description="Namespace for the second navigation camera topics",
            ),
            DeclareLaunchArgument(
                "rear_camera_namespace",
                default_value="/camera/rear",
                description="Namespace for the rear camera topics",
            ),
            DeclareLaunchArgument(
                "image_channel",
                default_value="0",
                description="Image channel index for UE",
            ),
            DeclareLaunchArgument(
                "depth_channel",
                default_value="100",
                description="Depth channel index for UE",
            ),
            DeclareLaunchArgument(
                "mask_channel",
                default_value="200",
                description="Mask channel index for UE",
            ),
            Node(
                package="rammp_prototype_gui",
                executable="GuiBridge",
                name="Gui_bridge_node",
                output="screen",
                emulate_tty=True,
                respawn=False,
                respawn_delay=2.0,
                parameters=[
                    {
                        "ue_host": LaunchConfiguration("ue_host"),
                        "use_shared_memory": LaunchConfiguration("use_shared_memory"),
                        "ue_preset": LaunchConfiguration("ue_preset"),
                        "wrist_camera_namespace": LaunchConfiguration(
                            "wrist_camera_namespace"
                        ),
                        "nav_camera_namespace_1": LaunchConfiguration(
                            "nav_camera_namespace_1"
                        ),
                        "nav_camera_namespace_2": LaunchConfiguration(
                            "nav_camera_namespace_2"
                        ),
                        "rear_camera_namespace": LaunchConfiguration(
                            "rear_camera_namespace"
                        ),
                        "image_channel": LaunchConfiguration("image_channel"),
                        "depth_channel": LaunchConfiguration("depth_channel"),
                        "mask_channel": LaunchConfiguration("mask_channel"),
                    }
                ],
            ),
        ]
    )
