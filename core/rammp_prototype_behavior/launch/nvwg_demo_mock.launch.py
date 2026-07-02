from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rammp_prototype_behavior",
                executable="mock_curb_detection",
                output="screen",
            ),
        ]
    )
