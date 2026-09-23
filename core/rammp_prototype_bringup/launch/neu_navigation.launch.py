from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # NEU curb detection is two script nodes with no launch file of their
    # own. Lifted verbatim from full.launch.py so sheppy can start the pair
    # as one node.
    return LaunchDescription(
        [
            Node(
                package="neu_navigation",
                executable="perception_curb_descent_detection_node.py",
                name="perception_curb_descent_detection_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="neu_navigation",
                executable="perception_curb_detection_node.py",
                name="perception_curb_detection_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
