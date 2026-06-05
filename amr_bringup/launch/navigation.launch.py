import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node


def bringup_params_file() -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "params", "amr.yaml")


def bringup_launch_file(filename: str) -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    bringup_params = bringup_params_file()
    mapping_mode = LaunchConfiguration("mapping_mode")
    navigation_only = LaunchConfiguration("navigation_only")
    use_mqtt_server = LaunchConfiguration("use_mqtt_server")
    bringup_delay_sec = LaunchConfiguration("bringup_delay_sec")
    navigation_start_delay_sec = LaunchConfiguration("navigation_start_delay_sec")

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file("localization.launch.py")),
        launch_arguments={"mapping_mode": mapping_mode}.items(),
    )
    mqtt_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_mqtt_server"),
                "launch",
                "amr_mqtt_server.launch.py",
            )
        ),
        condition=IfCondition(
            PythonExpression(
                ["'", use_mqtt_server, "' == 'true' and '", navigation_only, "' == 'false'"]
            )
        ),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_controller_server"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={"params_file": bringup_params}.items(),
    )
    recovery_server = LifecycleNode(
        package="amr_recovery_server",
        executable="amr_recovery_server",
        name="recovery_server",
        namespace="amr",
        output="screen",
        parameters=[bringup_params],
    )
    bt_navigator = LifecycleNode(
        package="amr_bt_navigator",
        executable="amr_bt_navigator",
        name="navigator",
        namespace="amr",
        output="screen",
        parameters=[bringup_params],
    )
    runtime_observation = Node(
        package="amr_runtime_observation",
        executable="amr_runtime_observation",
        name="runtime_observation",
        namespace="amr",
        output="screen",
        parameters=[bringup_params],
    )
    navigation_manager = Node(
        package="amr_lifecycle_manager",
        executable="amr_lifecycle_manager",
        name="navigation_manager",
        namespace="amr",
        output="screen",
        parameters=[
            bringup_params,
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping_mode",
                default_value="false",
                description="Run pure SLAM mapping mode and skip navigation bringup.",
            ),
            DeclareLaunchArgument(
                "navigation_only",
                default_value="false",
                description="When true, start only controller/recovery/navigator nodes. Use this when prerequisite localization/runtime inputs are already started elsewhere.",
            ),
            DeclareLaunchArgument(
                "use_mqtt_server",
                default_value="false",
                description="When true and navigation_only is false, also start the optional amr_mqtt_server bridge.",
            ),
            DeclareLaunchArgument(
                "bringup_delay_sec",
                default_value="0.0",
                description="Delay before AMR bringup starts.",
            ),
            DeclareLaunchArgument(
                "navigation_start_delay_sec",
                default_value="3.0",
                description="Additional delay before navigation nodes start after prerequisite bringup.",
            ),
            TimerAction(
                period=bringup_delay_sec,
                actions=[
                    localization_launch,
                    mqtt_server_launch,
                ],
                condition=UnlessCondition(navigation_only),
            ),
            TimerAction(
                period=PythonExpression(
                    [bringup_delay_sec, " + ", navigation_start_delay_sec]
                ),
                actions=[
                    controller_launch,
                    recovery_server,
                    bt_navigator,
                    runtime_observation,
                    navigation_manager,
                ],
                condition=UnlessCondition(mapping_mode),
            ),
        ]
    )
