from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    show_cv = LaunchConfiguration("show_opencv_windows")
    return LaunchDescription(
        [
            DeclareLaunchArgument("show_opencv_windows", default_value="false"),
            Node(
                package="cmu_door_opener",
                executable="button_detector",
                name="button_detector",
                output="screen",
                parameters=[
                    {
                        "show_opencv_windows": show_cv,
                        "process_rate_hz": 5.0,
                        "filter_alpha": 0.3,
                        "filter_min_samples": 3,
                    }
                ],
            ),
            Node(
                package="cmu_door_opener",
                executable="button_push_controller",
                name="button_push_controller",
                output="screen",
            ),
        ]
    )
