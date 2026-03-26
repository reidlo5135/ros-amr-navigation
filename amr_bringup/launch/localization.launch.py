import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def bringup_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "params", "amr.yaml")


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    mapping_mode = LaunchConfiguration("mapping_mode")
    startup_localization_mode = LaunchConfiguration("startup_localization_mode")

    ld.add_action(
        DeclareLaunchArgument(
            "mapping_mode",
            default_value="false",
            description="Run amr_map_server in live mapping mode and skip localization/global planner bringup.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "startup_localization_mode",
            default_value="global_relocalization",
            description=(
                "Startup localization policy: "
                "manual_set_initial_pose | global_relocalization | active_relocalization | fixed_start_pose"
            ),
        )
    )

    map_server = LifecycleNode(
        package="amr_map_server",
        executable="amr_map_server",
        name="map_server",
        namespace="amr",
        output="screen",
        parameters=[bringup_params_file(), {"mode.mapping": mapping_mode}],
    )
    localization = LifecycleNode(
        package="amr_localization",
        executable="amr_localization",
        name="localization",
        namespace="amr",
        output="screen",
        parameters=[
            bringup_params_file(),
            {"startup.localization_mode": startup_localization_mode},
        ],
        condition=UnlessCondition(mapping_mode),
    )
    global_planner = LifecycleNode(
        package="amr_global_planner",
        executable="amr_global_planner",
        name="global_planner",
        namespace="amr",
        output="screen",
        parameters=[bringup_params_file()],
        condition=UnlessCondition(mapping_mode),
    )
    costmap_server = LifecycleNode(
        package="amr_costmap_server",
        executable="amr_costmap_server",
        name="costmap_server",
        namespace="amr",
        output="screen",
        parameters=[bringup_params_file()],
        condition=UnlessCondition(mapping_mode),
    )

    localization_manager = Node(
        package="amr_lifecycle_manager",
        executable="amr_lifecycle_manager",
        name="localization_manager",
        namespace="amr",
        output="screen",
        parameters=[
            bringup_params_file(),
            {
                "managed_nodes": [
                    "/amr/map_server",
                    "/amr/localization",
                    "/amr/costmap_server",
                    "/amr/global_planner",
                ],
                "startup.localization_mode": startup_localization_mode,
            },
        ],
        condition=UnlessCondition(mapping_mode),
    )
    mapping_manager = Node(
        package="amr_lifecycle_manager",
        executable="amr_lifecycle_manager",
        name="localization_manager",
        namespace="amr",
        output="screen",
        parameters=[
            bringup_params_file(),
            {
                "managed_nodes": [
                    "/amr/map_server",
                ],
                "startup.localization_mode": startup_localization_mode,
                "initial_pose.enabled": False,
            },
        ],
        condition=IfCondition(mapping_mode),
    )

    ld.add_action(map_server)
    ld.add_action(localization)
    ld.add_action(costmap_server)
    ld.add_action(global_planner)
    ld.add_action(localization_manager)
    ld.add_action(mapping_manager)
    return ld
