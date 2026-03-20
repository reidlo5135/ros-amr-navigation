import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_mqtt_robot_plugin")
    return os.path.join(package_share_directory, "config", "amr_mqtt_robot_plugin.yaml")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file(),
                description="Parameter file for amr_mqtt_robot_plugin",
            ),
            Node(
                package="amr_mqtt_robot_plugin",
                executable="amr_mqtt_robot_plugin",
                name="mqtt_robot_plugin",
                namespace="amr",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            )
        ]
    )
