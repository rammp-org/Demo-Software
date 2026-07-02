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
    chair_ip_arg = DeclareLaunchArgument(
        "chair_ip",
        default_value="10.2.10.3",
        description="IP address of the LUCI chair gRPC interface",
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
    launch_luci_arg = DeclareLaunchArgument(
        "launch_luci",
        default_value="true",
        description="Launch LUCI gRPC interface node",
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
    launch_cmu_door_opener_arg = DeclareLaunchArgument(
        "launch_cmu_door_opener",
        default_value="true",
        description="Launch CMU door opener nodes (button_detector + button_push_controller)",
    )
    launch_cameras_arg = DeclareLaunchArgument(
        "launch_cameras",
        default_value="true",
        description="Launch camera nodes (RealSense wrist + Orbbec nav)",
    )
    # Future modules: flip default_value to "true" when ready to enable
    launch_neu_navigation_arg = DeclareLaunchArgument(
        "launch_neu_navigation",
        default_value="false",
        description="Launch NEU curb detection node",
    )
    launch_cornell_feeding_arg = DeclareLaunchArgument(
        "launch_cornell_feeding",
        default_value="false",
        description="Launch Cornell feeding module",
    )
    launch_atdev_coffee_stabilizer_arg = DeclareLaunchArgument(
        "launch_atdev_coffee_stabilizer",
        default_value="true",
        description="Launch ATDev cup stabilizer module",
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

    luci_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "luci.launch.py")
        ),
        launch_arguments={"chair_ip": LaunchConfiguration("chair_ip")}.items(),
        condition=IfCondition(LaunchConfiguration("launch_luci")),
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

    cmu_door_opener_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cmu_door_opener"),
                "launch",
                "cmu_door_opener.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_cmu_door_opener")),
    )

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

    # NEU navigation — curb detection (no dedicated launch file; installed as a script)
    neu_navigation_descent_node = Node(
        package="neu_navigation",
        executable="perception_curb_descent_detection_node.py",
        name="perception_curb_descent_detection_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("launch_neu_navigation")),
    )

    neu_navigation_ascent_node = Node(
        package="neu_navigation",
        executable="perception_curb_detection_node.py",
        name="perception_curb_detection_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("launch_neu_navigation")),
    )

    # Future: Cornell feeding module
    # cornell_feeding_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("cornell_feeding"),
    #             "launch",\
    #             "cornell_feeding.launch.py",
    #         )
    #     ),
    #     condition=IfCondition(LaunchConfiguration("launch_cornell_feeding")),
    # )

    atdev_coffee_stabilizer_node = Node(
        package="rammp_prototype_behavior",
        executable="mock_cup_stabilizer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("launch_atdev_coffee_stabilizer")),
    )

    return LaunchDescription(
        [
            # Arguments — hardware config
            serial_port_arg,
            chair_ip_arg,
            ue_host_arg,
            # Arguments — enable/disable flags
            launch_description_arg,
            launch_mebot_driver_arg,
            launch_luci_arg,
            launch_arm_driver_arg,
            launch_gui_bridge_arg,
            launch_system_control_arg,
            launch_cmu_door_opener_arg,
            launch_cameras_arg,
            launch_neu_navigation_arg,
            launch_cornell_feeding_arg,
            launch_atdev_coffee_stabilizer_arg,
            # Core infrastructure
            description_launch,
            mebot_driver_launch,
            luci_launch,
            arm_driver_node,
            gui_bridge_launch,
            system_control_node,
            # Demo modules
            cmu_door_opener_launch,
            cameras_launch,
            neu_navigation_descent_node,
            neu_navigation_ascent_node,
            atdev_coffee_stabilizer_node,
        ]
    )
