from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    enable_robot_model = LaunchConfiguration('enable_robot_model')
    enable_robot_meshes = LaunchConfiguration('enable_robot_meshes')
    robot_model_renderer_backend = LaunchConfiguration('robot_model_renderer_backend')
    enable_map_visualization = LaunchConfiguration('enable_map_visualization')
    enable_costmap_visualization = LaunchConfiguration('enable_costmap_visualization')
    enable_scan_visualization = LaunchConfiguration('enable_scan_visualization')
    enable_tf_visualization = LaunchConfiguration('enable_tf_visualization')
    robot_opengl_target_fps = LaunchConfiguration('robot_opengl_target_fps')
    robot_opengl_software_target_fps = LaunchConfiguration('robot_opengl_software_target_fps')
    robot_opengl_hardware_target_fps = LaunchConfiguration('robot_opengl_hardware_target_fps')
    robot_opengl_auto_software_profile = LaunchConfiguration('robot_opengl_auto_software_profile')
    robot_model_pose_epsilon_m = LaunchConfiguration('robot_model_pose_epsilon_m')
    robot_model_yaw_epsilon_rad = LaunchConfiguration('robot_model_yaw_epsilon_rad')
    mesh_max_loaded_triangles = LaunchConfiguration('mesh_max_loaded_triangles')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('amr_visualization'),
                'config',
                'low_cpu_opengl.yaml',
            ]),
            description='Parameter file for amr_visualization.',
        ),
        DeclareLaunchArgument(
            'enable_robot_model',
            default_value='true',
            description='Enable the robot model/overlay pipeline.',
        ),
        DeclareLaunchArgument(
            'enable_robot_meshes',
            default_value='true',
            description='Enable loading and rendering robot mesh geometry.',
        ),
        DeclareLaunchArgument(
            'robot_model_renderer_backend',
            default_value='opengl',
            description='Robot renderer backend: proxy, qpainter_wireframe, or opengl.',
        ),
        DeclareLaunchArgument(
            'enable_map_visualization',
            default_value='true',
            description='Enable map visualization and subscription.',
        ),
        DeclareLaunchArgument(
            'enable_costmap_visualization',
            default_value='true',
            description='Enable global/local costmap visualization and subscriptions.',
        ),
        DeclareLaunchArgument(
            'enable_scan_visualization',
            default_value='true',
            description='Enable scan visualization and subscription.',
        ),
        DeclareLaunchArgument(
            'enable_tf_visualization',
            default_value='true',
            description='Enable TF frame visualization. Robot model TF resolution remains available.',
        ),
        DeclareLaunchArgument(
            'robot_opengl_target_fps',
            default_value='0',
            description='Explicit OpenGL FPS cap. Use 0 to auto-select the software/hardware profile.',
        ),
        DeclareLaunchArgument(
            'robot_opengl_software_target_fps',
            default_value='8',
            description='OpenGL FPS cap used when a software renderer such as llvmpipe is detected.',
        ),
        DeclareLaunchArgument(
            'robot_opengl_hardware_target_fps',
            default_value='30',
            description='OpenGL FPS cap used for hardware OpenGL when no explicit cap is set.',
        ),
        DeclareLaunchArgument(
            'robot_opengl_auto_software_profile',
            default_value='true',
            description='Automatically switch to the software OpenGL FPS profile for llvmpipe/softpipe.',
        ),
        DeclareLaunchArgument(
            'robot_model_pose_epsilon_m',
            default_value='0.003',
            description='Robot visual translation epsilon used to suppress tiny TF-only model updates.',
        ),
        DeclareLaunchArgument(
            'robot_model_yaw_epsilon_rad',
            default_value='0.003',
            description='Robot visual angular epsilon used to suppress tiny TF-only model updates.',
        ),
        DeclareLaunchArgument(
            'mesh_max_loaded_triangles',
            default_value='200000',
            description='Maximum STL triangles loaded per mesh; low values may remove visible robot detail.',
        ),
        Node(
            package='amr_visualization',
            executable='amr_visualization',
            name='amr_visualization',
            output='screen',
            parameters=[
                params_file,
                {
                    'enable_robot_model': ParameterValue(enable_robot_model, value_type=bool),
                    'enable_robot_meshes': ParameterValue(enable_robot_meshes, value_type=bool),
                    'robot_model_renderer_backend': ParameterValue(robot_model_renderer_backend, value_type=str),
                    'enable_map_visualization': ParameterValue(enable_map_visualization, value_type=bool),
                    'enable_costmap_visualization': ParameterValue(enable_costmap_visualization, value_type=bool),
                    'enable_scan_visualization': ParameterValue(enable_scan_visualization, value_type=bool),
                    'enable_tf_visualization': ParameterValue(enable_tf_visualization, value_type=bool),
                    'robot_opengl_target_fps': ParameterValue(robot_opengl_target_fps, value_type=int),
                    'robot_opengl_software_target_fps': ParameterValue(
                        robot_opengl_software_target_fps,
                        value_type=int,
                    ),
                    'robot_opengl_hardware_target_fps': ParameterValue(
                        robot_opengl_hardware_target_fps,
                        value_type=int,
                    ),
                    'robot_opengl_auto_software_profile': ParameterValue(
                        robot_opengl_auto_software_profile,
                        value_type=bool,
                    ),
                    'robot_model_pose_epsilon_m': ParameterValue(robot_model_pose_epsilon_m, value_type=float),
                    'robot_model_yaw_epsilon_rad': ParameterValue(robot_model_yaw_epsilon_rad, value_type=float),
                    'mesh_max_loaded_triangles': ParameterValue(mesh_max_loaded_triangles, value_type=int),
                },
            ],
        ),
    ])
