from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_directory",
                default_value="~/bags",
                description="Directory to save the bags",
            ),
            Node(
                package="data_logger",
                executable="ros_bag_node",
                name="ros_bag_node",
                output="screen",
                parameters=[{"bag_directory": LaunchConfiguration("bag_directory")}],
            ),
        ]
    )
