from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
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
