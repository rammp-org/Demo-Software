from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="base",
                description="Top-level namespace for the node",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Serial port for Teensy connection (e.g. /dev/ttyACM0 or /dev/rfcomm0)",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="ROS 2 log level (DEBUG, INFO, WARN, ERROR, FATAL)",
            ),
            Node(
                package="rammp_prototype_driver",
                executable="control_node",
                name="base_control_node",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                    }
                ],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )
