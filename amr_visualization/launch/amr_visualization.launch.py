"""@file amr_visualization.launch.py
@brief Launch the AMR Qt visualization application.
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'with_mcp_server',
            default_value='false',
            description='Launch amr_mcp_server together with AMR Visualization.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('amr_mcp_server'),
                    'launch',
                    'amr_mcp_server.launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('with_mcp_server')),
        ),
        Node(
            package='amr_visualization',
            executable='amr_visualization',
            name='amr_visualization',
            output='screen',
        ),
    ])
