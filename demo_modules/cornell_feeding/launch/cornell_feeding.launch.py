"""
Launch the cornell_feeding drink action server (RAMMP's drink node via the shim).

Mirrors RAMMP's minimal launch: a map->world static TF + the drink_action_server. Use
run_on_robot:=true on the real robot (drives the shared arm_driver via /arm/* and uses the
RealSense); default is sim (no hardware). Replaces rammp_prototype_behavior's
mock_drinking_node in bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _drink_node(context, *args, **kwargs):
    """
    Build the drink_action_server Node with args resolved at launch time.

    The --run_on_robot flag is appended only when requested; passing it as a
    conditional substitution would otherwise inject an empty-string argument
    that the node's argparse rejects.
    """
    scene_config = LaunchConfiguration("scene_config").perform(context)
    run_on_robot = LaunchConfiguration("run_on_robot").perform(context)

    node_args = ["--scene_config", scene_config]
    if run_on_robot.lower() in ("true", "1"):
        node_args.append("--run_on_robot")

    return [
        Node(
            package="cornell_feeding",
            executable="drink_action_server",
            name="drink_action_server",
            output="screen",
            arguments=node_args,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value="wheelchair"),
        DeclareLaunchArgument("run_on_robot", default_value="false"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="st_map2world",
            arguments=["0", "0", "0", "0", "0", "0", "1", "map", "world"],
        ),
        OpaqueFunction(function=_drink_node),
    ])
