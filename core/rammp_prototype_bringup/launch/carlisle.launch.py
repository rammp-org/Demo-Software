import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("rammp_prototype_bringup")

    # ── Launch arguments ──────────────────────────────────────────────────────

    # Hardware config
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for MEBot Teensy connection",
    )
    ue_host_arg = DeclareLaunchArgument(
        "ue_host",
        default_value="127.0.0.1",
        description="IP address of the Unreal Engine host",
    )

    # Per-node enable flags
    launch_description_arg = DeclareLaunchArgument(
        "launch_description",
        default_value="true",
        description="Launch robot_state_publisher and wrist camera static TF",
    )
    launch_mebot_driver_arg = DeclareLaunchArgument(
        "launch_mebot_driver",
        default_value="true",
        description="Launch MEBot Teensy control node",
    )
    launch_arm_driver_arg = DeclareLaunchArgument(
        "launch_arm_driver",
        default_value="true",
        description="Launch Kinova arm driver node",
    )
    launch_gui_bridge_arg = DeclareLaunchArgument(
        "launch_gui_bridge",
        default_value="true",
        description="Launch GUI bridge node (required by system_control)",
    )
    launch_system_control_arg = DeclareLaunchArgument(
        "launch_system_control",
        default_value="true",
        description="Launch behavior state machine node",
    )
    launch_cameras_arg = DeclareLaunchArgument(
        "launch_cameras",
        default_value="true",
        description="Launch camera nodes (RealSense wrist + Orbbec nav)",
    )

    # ── Core infrastructure ──────────────────────────────────────+─────────────

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rammp_prototype_description"),
                "launch",
                "description.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_description")),
    )

    mebot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rammp_prototype_driver"),
                "launch",
                "control_node.launch.py",
            )
        ),
        launch_arguments={"serial_port": LaunchConfiguration("serial_port")}.items(),
        condition=IfCondition(LaunchConfiguration("launch_mebot_driver")),
    )

    arm_driver_node = Node(
        package="arm_driver",
        executable="arm_driver",
        name="arm_driver_node",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("launch_arm_driver")),
    )

    gui_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rammp_prototype_gui"),
                "launch",
                "Gui_bridge.launch.py",
            )
        ),
        launch_arguments={"ue_host": LaunchConfiguration("ue_host")}.items(),
        condition=IfCondition(LaunchConfiguration("launch_gui_bridge")),
    )

    system_control_node = Node(
        package="rammp_prototype_behavior",
        executable="system_control",
        name="system_control",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("launch_system_control")),
    )

    # ── Demo modules ──────────────────────────────────────────────────────────

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "camera.launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(
                get_package_share_directory("rammp_prototype_bringup"),
                "config",
                "camera_demo_main.yaml",
            )
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_cameras")),
    )

    return LaunchDescription(
        [
            # Arguments — hardware config
            serial_port_arg,
            ue_host_arg,
            # Arguments — enable/disable flags
            launch_description_arg,
            launch_mebot_driver_arg,
            launch_arm_driver_arg,
            launch_gui_bridge_arg,
            launch_system_control_arg,
            launch_cameras_arg,
            # Core infrastructure
            description_launch,
            mebot_driver_launch,
            arm_driver_node,
            gui_bridge_launch,
            system_control_node,
            # Demo modules
            cameras_launch,
        ]
    )
