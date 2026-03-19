from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    chair_ip_arg = DeclareLaunchArgument(
        "chair_ip",
        default_value="192.168.0.112",  # IP address of HERL LUCI chair
        description="IP address of the LUCI chair",
    )

    luci_grpc_node = Node(
        package="luci_grpc_interface",
        executable="grpc_interface_node",
        name="luci_grpc_interface_node",
        arguments=["-a", LaunchConfiguration("chair_ip"), "--"],
        output="screen",
    )

    return LaunchDescription(
        [
            chair_ip_arg,
            luci_grpc_node,
        ]
    )
