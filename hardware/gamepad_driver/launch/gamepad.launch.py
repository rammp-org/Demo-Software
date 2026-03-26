from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rammp_protoype_description"),
                "launch",
                "description.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            description_launch,
            Node(
                package="arm_driver",
                executable="arm_driver",
                name="arm_driver",
                output="screen",
            ),
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
            ),
        ]
    )
