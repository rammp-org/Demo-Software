from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="xbox_controller_driver",
                executable="xbox_controller_node",
                name="xbox_controller_node",
                output="screen",
            )
        ]
    )
