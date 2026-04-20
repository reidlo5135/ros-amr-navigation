import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node


def bringup_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "params", "amr.yaml")


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_controller_server"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={"params_file": bringup_params_file()}.items(),
    )
    recovery_server = LifecycleNode(
        package="amr_recovery_server",
        executable="amr_recovery_server",
        name="recovery_server",
        namespace="amr",
        output="screen",
        parameters=[bringup_params_file()],
    )
    bt_navigator = LifecycleNode(
        package="amr_bt_navigator",
        executable="amr_bt_navigator",
        name="navigator",
        namespace="amr",
        output="screen",
        parameters=[bringup_params_file()],
    )
    runtime_observation = Node(
        package="amr_runtime_observation",
        executable="amr_runtime_observation",
        name="runtime_observation",
        namespace="amr",
        output="screen",
        parameters=[bringup_params_file()],
    )
    navigation_manager = Node(
        package="amr_lifecycle_manager",
        executable="amr_lifecycle_manager",
        name="navigation_manager",
        namespace="amr",
        output="screen",
        parameters=[
            bringup_params_file(),
            {
                "managed_nodes": [
                    "/amr/local_planner",
                    "/amr/motion_controller",
                    "/amr/recovery_server",
                    "/amr/navigator",
                ],
                "initial_pose.enabled": False,
            },
        ],
    )

    ld.add_action(controller_launch)
    ld.add_action(recovery_server)
    ld.add_action(bt_navigator)
    ld.add_action(runtime_observation)
    ld.add_action(navigation_manager)
    return ld
