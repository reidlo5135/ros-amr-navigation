import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def bringup_launch_file(filename: str) -> str:
    package_share_directory = get_package_share_directory("amr_bringup")
    return os.path.join(package_share_directory, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    mapping_mode = LaunchConfiguration("mapping_mode")
    navigation_start_delay_sec = LaunchConfiguration("navigation_start_delay_sec")

    ld.add_action(
        DeclareLaunchArgument(
            "mapping_mode",
            default_value="false",
            description="Run localization launch in mapping mode and skip delayed navigation bringup.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "navigation_start_delay_sec",
            default_value="3.0",
            description=(
                "Delay before starting navigation bringup so localization lifecycle "
                "activation and automatic initial pose publication complete first."
            ),
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file("localization.launch.py")),
        launch_arguments={"mapping_mode": mapping_mode}.items(),
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file("navigation.launch.py")),
        condition=UnlessCondition(mapping_mode),
    )
    delayed_navigation_launch = TimerAction(
        period=navigation_start_delay_sec,
        actions=[navigation_launch],
        condition=UnlessCondition(mapping_mode),
    )

    ld.add_action(localization_launch)
    ld.add_action(delayed_navigation_launch)
    return ld
