import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def bringup_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "params", "amr.yaml")


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    local_planner = LifecycleNode(
        package="amr_local_planner",
        executable="amr_local_planner",
        name="local_planner",
        namespace="amr",
        output="screen",
        parameters=[bringup_params_file()],
    )
    motion_controller = LifecycleNode(
        package="amr_motion_controller",
        executable="amr_motion_controller",
        name="motion_controller",
        namespace="amr",
        output="screen",
        parameters=[bringup_params_file()],
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
    rviz_bridge = Node(
        package="amr_rviz_plugins",
        executable="amr_goal_bridge",
        name="rviz_bridge",
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

    ld.add_action(local_planner)
    ld.add_action(motion_controller)
    ld.add_action(recovery_server)
    ld.add_action(bt_navigator)
    ld.add_action(rviz_bridge)
    ld.add_action(navigation_manager)
    return ld
