import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    turtlebot3_robot_launch = os.path.join(
        get_package_share_directory("turtlebot3_bringup"),
        "launch",
        "robot.launch.py",
    )
    mqtt_robot_plugin_launch = os.path.join(
        get_package_share_directory("amr_mqtt_robot_plugin"),
        "launch",
        "amr_mqtt_robot_plugin.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(turtlebot3_robot_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mqtt_robot_plugin_launch),
            ),
        ]
    )
