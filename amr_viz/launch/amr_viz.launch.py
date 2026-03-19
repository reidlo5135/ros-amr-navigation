import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bridge_config_file() -> str:
    package_share_directory = get_package_share_directory("amr_viz")
    return os.path.join(package_share_directory, "config", "bridge.yaml")


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    websocket_host = LaunchConfiguration("websocket_host")
    websocket_port = LaunchConfiguration("websocket_port")

    ld.add_action(
        DeclareLaunchArgument(
            "websocket_host",
            default_value="0.0.0.0",
            description="WebSocket bind host for the AMR visualization bridge.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "websocket_port",
            default_value="8765",
            description="WebSocket bind port for the AMR visualization bridge.",
        )
    )

    ld.add_action(
        Node(
            package="amr_viz",
            executable="amr_viz_bridge",
            name="viz_bridge",
            namespace="amr",
            output="screen",
            parameters=[
                bridge_config_file(),
                {
                    "websocket.host": websocket_host,
                    "websocket.port": websocket_port,
                },
            ],
        )
    )
    return ld
