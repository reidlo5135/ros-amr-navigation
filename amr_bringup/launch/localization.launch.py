import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def bringup_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "params", "amr.yaml")


def add_lifecycle_node(
    ld: LaunchDescription,
    node: LifecycleNode,
    condition=None,
) -> None:
    ld.add_action(node)
    ld.add_action(
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lambda action: action == node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=condition,
        )
    )
    ld.add_action(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=lambda action: action == node,
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            ),
            condition=condition,
        ),
    )


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    mapping_mode = LaunchConfiguration("mapping_mode")

    ld.add_action(
        DeclareLaunchArgument(
            "mapping_mode",
            default_value="false",
            description="Run amr_map_server in live mapping mode and skip localization/global planner bringup.",
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
        parameters=[bringup_params_file()],
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

    add_lifecycle_node(ld, map_server)
    add_lifecycle_node(ld, localization, UnlessCondition(mapping_mode))
    add_lifecycle_node(ld, global_planner, UnlessCondition(mapping_mode))
    return ld
