from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="rammp_prototype_behavior",
            #     executable="mock_arm_driver",
            #     output="screen",
            # ),
            Node(
                package="rammp_prototype_behavior",
                executable="mock_drinking_node",
                output="screen",
            ),
            Node(
                package="rammp_prototype_behavior",
                executable="mock_opening_door",
                output="screen",
            ),
            Node(
                package="rammp_prototype_behavior",
                executable="mock_cup_stabilizer",
                output="screen",
            ),
            # Node(
            #     package="rammp_prototype_behavior",
            #     executable="mock_chair_control",
            #     output="screen",
            # ),
            Node(
                package="rammp_prototype_behavior",
                executable="mock_curb_detection",
                output="screen",
            ),
        ]
    )
