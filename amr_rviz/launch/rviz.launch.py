import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_rviz_config() -> str:
    return os.path.join(
        get_package_share_directory("amr_rviz"),
        "rviz",
        "amr_light.rviz",
    )


def generate_launch_description() -> LaunchDescription:
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    goal_topic = LaunchConfiguration("goal_topic")
    goals_topic = LaunchConfiguration("goals_topic")
    navigate_to_pose_action = LaunchConfiguration("navigate_to_pose_action")
    navigate_to_poses_action = LaunchConfiguration("navigate_to_poses_action")
    default_frame_id = LaunchConfiguration("default_frame_id")

    bridge = Node(
        package="amr_rviz",
        executable="amr_rviz_bridge",
        name="rviz_bridge",
        namespace="amr",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "goal_topic": goal_topic,
                "goals_topic": goals_topic,
                "navigate_to_pose_action": navigate_to_pose_action,
                "navigate_to_poses_action": navigate_to_poses_action,
                "default_frame_id": default_frame_id,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config(),
                description="RViz config file to load.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if provided by the runtime.",
            ),
            DeclareLaunchArgument(
                "goal_topic",
                default_value="/amr/rviz/goal",
                description="PoseStamped topic consumed from RViz 2D Goal Pose.",
            ),
            DeclareLaunchArgument(
                "goals_topic",
                default_value="/amr/rviz/goals",
                description="PoseArray topic used for multi-goal route dispatch.",
            ),
            DeclareLaunchArgument(
                "navigate_to_pose_action",
                default_value="/amr/navigator/navigate_to_pose",
                description="Single-goal navigation action name.",
            ),
            DeclareLaunchArgument(
                "navigate_to_poses_action",
                default_value="/amr/navigator/navigate_to_poses",
                description="Waypoint-route navigation action name.",
            ),
            DeclareLaunchArgument(
                "default_frame_id",
                default_value="map",
                description="Fallback frame_id when incoming RViz messages omit one.",
            ),
            bridge,
            rviz,
        ]
    )
