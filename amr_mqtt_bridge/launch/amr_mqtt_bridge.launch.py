from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
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
