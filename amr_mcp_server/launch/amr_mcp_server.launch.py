"""Launch the ROS-native AMR MCP server."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('amr_mcp_server'),
        'config',
        'amr_mcp_server.yaml',
    ])

    return LaunchDescription([
        Node(
            package='amr_mcp_server',
            executable='amr_mcp_server',
            name='amr_mcp_server',
            output='screen',
            parameters=[params],
        ),
    ])
