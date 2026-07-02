"""@file controller.launch.py
@brief Launch the AMR controller server with a configurable params file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    controller_server = Node(
        package="amr_controller_server",
        executable="amr_controller_server",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="Controller-layer parameter file provided by amr_bringup.",
            ),
            controller_server,
        ]
    )
