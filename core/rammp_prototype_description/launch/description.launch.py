import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("kortex_description"),
        "robots",
        "gen3.xacro",
    )
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " dof:=7 gripper:=robotiq_2f_85"]),
        value_type=str,
    )

    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame",
        default_value="wrist_wrist_camera_link",
        description="Child frame ID for the wrist camera static transform",
    )

    return LaunchDescription(
        [
            camera_frame_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", "/arm/joint_states")],
            ),
            # Static transform from the arm end-effector to the wrist camera mount.
            # Translation (x=0.01, y=0.0615, z=0.03) is the measured offset in metres
            # from end_effector_link to the camera optical centre on the physical mount.
            # Quaternion (0.5, 0.5, 0.5, -0.5) rotates the camera frame so that its
            # Z-axis points forward (optical axis) and X-axis points right, aligning
            # with the ROS camera convention (rpy ~ [-pi/2, 0, pi/2]).
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
                    "0.5",
                    "--qz",
                    "0.5",
                    "--qw",
                    "-0.5",
                    "--frame-id",
                    "end_effector_link",
                    "--child-frame-id",
                    LaunchConfiguration("camera_frame"),
                ],
            ),
        ]
    )
