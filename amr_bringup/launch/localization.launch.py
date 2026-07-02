"""@file localization.launch.py
@brief Deprecated localization launch shim that points users to slam_toolbox navigation.
"""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "amr_bringup localization.launch.py is deprecated. "
                    "Run robot bringup and slam_toolbox externally, then launch "
                    "amr_bringup navigation.launch.py."
                )
            )
        ]
    )
