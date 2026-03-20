import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("kortex_description"),
        "robots",
        "gen3.xacro",
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " dof:=7",
                " gripper:=robotiq_2f_85",
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", "/arm/joint_states")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.01",
                    "--y",
                    "0.0615",
                    "--z",
                    "0.03",
                    "--qx",
                    "0.5",
                    "--qy",
                    "-0.5",
                    "--qz",
                    "-0.5",
                    "--qw",
                    "0.5",
                    "--frame-id",
                    "end_effector_link",
                    "--child-frame-id",
                    "camera_link",
                ],
            ),
        ]
    )
