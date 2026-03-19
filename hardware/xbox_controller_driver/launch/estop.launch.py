from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="xbox_controller_driver",
                executable="estop_node",
                name="estop_node",
                output="screen",
            )
        ]
    )
