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
                        # Empty device_path => auto-select the node that advertises
                        # the target keys. Set an explicit /dev/input/eventX to pin
                        # it. device_name is only a fallback substring filter.
                        "device_path": "",
                        "device_name": "",
                        "grab_device": False,
                    }
                ],
            ),
        ]
    )
