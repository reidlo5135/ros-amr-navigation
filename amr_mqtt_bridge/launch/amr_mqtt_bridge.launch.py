import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_mqtt_bridge")
    return os.path.join(package_share_directory, "config", "amr_mqtt_bridge.yaml")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file(),
                description="Parameter file for amr_mqtt_bridge",
            ),
            Node(
                package="amr_mqtt_bridge",
                executable="amr_mqtt_bridge",
                name="mqtt_bridge",
                namespace="amr",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            )
        ]
    )
