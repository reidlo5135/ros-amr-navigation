from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os;
import launch;
import launch_ros.actions;
from ament_index_python.packages import get_package_share_directory;
from launch.actions import IncludeLaunchDescription;
from launch.launch_description_sources import PythonLaunchDescriptionSource;


def generate_launch_description() -> LaunchDescription:

    package_name: str = "amr_mqtt_server";
    package_shared_directory: str = get_package_share_directory(package_name);
    parameter: str = os.path.join(package_shared_directory, "config", f"{package_name}.yaml");

    return LaunchDescription(
        [
            Node(
                package="amr_mqtt_server",
                executable="amr_mqtt_server",
                name="mqtt_server",
                namespace="amr",
                output="screen",
                parameters=[parameter],
            )
        ]
    )
