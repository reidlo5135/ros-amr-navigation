"""@file amr_visualization.launch.py
@brief Launch the AMR Qt visualization application.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_visualization',
            executable='amr_visualization',
            name='amr_visualization',
            output='screen',
        ),
    ])
