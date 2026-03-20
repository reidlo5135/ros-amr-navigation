import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def bringup_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "params", "amr.yaml")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="amr_robot_mqtt_bridge",
                executable="amr_robot_mqtt_bridge",
                name="robot_mqtt_bridge",
                namespace="amr",
                output="screen",
                parameters=[bringup_params_file()],
            )
        ]
    )
