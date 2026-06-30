from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="keyboard_driver",
                executable="keyboard_node",
                name="keyboard_node",
                output="screen",
                parameters=[
                    {
                        # Leave device_path empty to auto-match by name; or set an
                        # explicit /dev/input/eventX path.
                        "device_path": "",
                        "device_name": "keyboard",
                        "grab_device": False,
                    }
                ],
            ),
        ]
    )
