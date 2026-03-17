import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def bringup_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "params", "amr.yaml")


def add_lifecycle_node(ld: LaunchDescription, node: LifecycleNode) -> None:
    ld.add_action(node)
    ld.add_action(
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lambda action: action == node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
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
            )
        )
    )


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

    add_lifecycle_node(ld, local_planner)
    add_lifecycle_node(ld, motion_controller)
    add_lifecycle_node(ld, bt_navigator)
    ld.add_action(rviz_bridge)
    return ld
