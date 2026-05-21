from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="gamepad_driver",
                executable="gamepad_node",
                name="gamepad_node",
                output="screen",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                # Configure autorepeat
                parameters=[{"autorepeat_rate": 40.0}],
            ),
        ]
    )
