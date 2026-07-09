"""
Launch the AMR spatial segmenter lifecycle node.

This file starts the spatial segmenter as a standalone lifecycle node.
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            LifecycleNode(
                package="amr_spatial_segmenter",
                executable="amr_spatial_segmenter",
                name="spatial_segmenter",
                namespace="",
                output="screen",
            ),
        ]
    )
