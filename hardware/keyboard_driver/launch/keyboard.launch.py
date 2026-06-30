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
                        # Stable symlink from udev/99-mebot-keypad.rules. If it's
                        # not installed, the node warns and falls back to
                        # auto-selecting the node(s) that advertise the target keys.
                        "device_path": "/dev/mebot_keypad",
                        "device_name": "",
                        "grab_device": False,
                    }
                ],
            ),
        ]
    )
