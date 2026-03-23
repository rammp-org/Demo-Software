from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # kortex_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("kortex_bringup"),
    #             "launch",
    #             "gen3.launch.py",
    #         )
    #     ),
    #     launch_arguments={
    #         "robot_ip": "192.168.1.10",
    #         "dof": "6",
    #         "gripper": "robotiq_2f_85",
    #     }.items(),
    # )

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
            ),
        ]
    )
