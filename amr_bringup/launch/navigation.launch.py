import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def bringup_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "params", "amr.yaml")


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    use_mqtt_server = LaunchConfiguration("use_mqtt_server")

    costmap_server = LifecycleNode(
        package="amr_costmap_server",
        executable="amr_costmap_server",
        name="costmap_server",
        namespace="",
        output="screen",
        parameters=[params_file],
    )
    global_planner = LifecycleNode(
        package="amr_global_planner",
        executable="amr_global_planner",
        name="global_planner",
        namespace="",
        output="screen",
        parameters=[params_file],
    )
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_controller_server"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={"params_file": params_file}.items(),
    )
    recovery_server = LifecycleNode(
        package="amr_recovery_server",
        executable="amr_recovery_server",
        name="recovery_server",
        namespace="",
        output="screen",
        parameters=[params_file],
    )
    bt_navigator = LifecycleNode(
        package="amr_bt_navigator",
        executable="amr_bt_navigator",
        name="navigator",
        namespace="",
        output="screen",
        parameters=[params_file],
    )
    runtime_observation = Node(
        package="amr_runtime_observation",
        executable="amr_runtime_observation",
        name="runtime_observation",
        output="screen",
        parameters=[params_file],
    )
    lifecycle_manager = Node(
        package="amr_lifecycle_manager",
        executable="amr_lifecycle_manager",
        name="navigation_manager",
        output="screen",
        parameters=[
            params_file,
            {
                "managed_nodes": [
                    "/costmap_server",
                    "/global_planner",
                    "/local_planner",
                    "/motion_controller",
                    "/recovery_server",
                    "/navigator",
                ],
                "initial_pose.enabled": False,
            },
        ],
    )
    mqtt_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_mqtt_server"),
                "launch",
                "amr_mqtt_server.launch.py",
            )
        ),
        condition=IfCondition(use_mqtt_server),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=bringup_params_file(),
                description="Navigation parameter file.",
            ),
            DeclareLaunchArgument(
                "use_mqtt_server",
                default_value="false",
                description="Start the optional MQTT bridge with the navigation stack.",
            ),
            costmap_server,
            global_planner,
            controller_launch,
            recovery_server,
            bt_navigator,
            runtime_observation,
            lifecycle_manager,
            mqtt_server_launch,
        ]
    )
