import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def bringup_launch_file(filename: str) -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    bringup_params = os.path.join(
        get_package_share_directory("amr_bringup"),
        "params",
        "amr.yaml",
    )
    turtlebot3_robot_launch = os.path.join(
        get_package_share_directory("turtlebot3_bringup"),
        "launch",
        "robot.launch.py",
    )
    mqtt_bridge_launch = os.path.join(
        get_package_share_directory("amr_mqtt_bridge"),
        "launch",
        "amr_mqtt_bridge.launch.py",
    )

    mapping_mode = LaunchConfiguration("mapping_mode")
    robot_bringup_delay_sec = LaunchConfiguration("robot_bringup_delay_sec")
    navigation_start_delay_sec = LaunchConfiguration("navigation_start_delay_sec")
    startup_localization_mode = LaunchConfiguration("startup_localization_mode")

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file("localization.launch.py")),
        launch_arguments={
            "mapping_mode": mapping_mode,
            "startup_localization_mode": startup_localization_mode,
        }.items(),
    )
    mqtt_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mqtt_bridge_launch),
        launch_arguments={"params_file": bringup_params}.items(),
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file("navigation.launch.py")),
        condition=UnlessCondition(mapping_mode),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping_mode",
                default_value="false",
                description="Run localization in mapping mode and skip navigation bringup.",
            ),
            DeclareLaunchArgument(
                "robot_bringup_delay_sec",
                default_value="3.0",
                description="Delay after turtlebot3_bringup robot.launch.py before AMR stacks start.",
            ),
            DeclareLaunchArgument(
                "navigation_start_delay_sec",
                default_value="3.0",
                description="Additional delay before navigation starts after localization bringup.",
            ),
            DeclareLaunchArgument(
                "startup_localization_mode",
                default_value="global_relocalization",
                description=(
                    "Startup localization policy: "
                    "manual_set_initial_pose | global_relocalization | active_relocalization | fixed_start_pose"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(turtlebot3_robot_launch),
            ),
            TimerAction(
                period=robot_bringup_delay_sec,
                actions=[
                    localization_launch,
                    mqtt_bridge,
                ],
            ),
            TimerAction(
                period=PythonExpression(
                    [robot_bringup_delay_sec, " + ", navigation_start_delay_sec]
                ),
                actions=[navigation_launch],
                condition=UnlessCondition(mapping_mode),
            ),
        ]
    )
