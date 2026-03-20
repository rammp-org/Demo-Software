from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rammp_prototype_driver",
                executable="control_node",
                name="MEBot_control_node",
                namespace="base",
                output="screen",
            )
        ]
    )
