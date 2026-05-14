import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description() -> LaunchDescription:
    bringup_params = os.path.join(
        get_package_share_directory("amr_bringup"),
        "params",
        "amr.yaml",
    )
    description_file = LaunchConfiguration("description_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_prefix = LaunchConfiguration("frame_prefix")
    use_base_driver = LaunchConfiguration("use_base_driver")
    use_lidar_driver = LaunchConfiguration("use_lidar_driver")
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")
    lidar_backend = LaunchConfiguration("lidar_backend")
    robot_type = LaunchConfiguration("robot_type")
    robot_model = LaunchConfiguration("robot_model")
    base_port = LaunchConfiguration("base_port")
    base_baudrate = LaunchConfiguration("base_baudrate")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    sensor_model = LaunchConfiguration("sensor_model")
    publish_tf = LaunchConfiguration("publish_tf")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    robot_description = ParameterValue(
        Command(["xacro", " ", description_file, " ", "prefix:=", frame_prefix]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="description",
        namespace="amr",
        output="screen",
        parameters=[
            bringup_params,
            {
                "robot.type": robot_type,
                "robot.model": robot_model,
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
                "frame_prefix": frame_prefix,
            },
        ],
        condition=IfCondition(use_robot_state_publisher),
    )

    base_driver = Node(
        package="amr_bringup",
        executable="amr_bringup_hardware",
        name="base_driver",
        namespace="amr",
        output="screen",
        parameters=[
            bringup_params,
            {
                "bringup.component": "base_driver",
                "robot.type": robot_type,
                "robot.model": robot_model,
                "serial.port": base_port,
                "serial.baudrate": base_baudrate,
                "topics.cmd_vel": cmd_vel_topic,
                "topics.odom": odom_topic,
                "topics.imu": imu_topic,
                "topics.joint_states": joint_states_topic,
                "publish_tf": publish_tf,
                "use_sim_time": use_sim_time,
            },
        ],
        condition=IfCondition(use_base_driver),
    )

    lidar_driver = Node(
        package="amr_bringup",
        executable="amr_bringup_hardware",
        name="lidar_driver",
        namespace="amr",
        output="screen",
        parameters=[
            bringup_params,
            {
                "bringup.component": "lidar_driver",
                "robot.type": robot_type,
                "robot.model": robot_model,
                "serial.port": lidar_port,
                "serial.baudrate": lidar_baudrate,
                "topic": scan_topic,
                "sensor_model": sensor_model,
                "use_sim_time": use_sim_time,
            },
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", use_lidar_driver, "' == 'true' and '", lidar_backend, "' == 'internal'"]
            )
        ),
    )

    external_lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hls_lfcd_lds_driver"),
                "launch",
                "hlds_laser.launch.py",
            )
        ),
        launch_arguments={
            "port": lidar_port,
            "frame_id": "base_scan",
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_lidar_driver,
                    "' == 'true' and '",
                    lidar_backend,
                    "' == 'external_hlds'",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "robot_type",
                default_value="turtlebot3",
                description="Robot type selector for the AMR-owned hardware entrypoint.",
            ),
            DeclareLaunchArgument(
                "robot_model",
                default_value="burger",
                description="Robot model selector for the AMR-owned hardware entrypoint.",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value=os.path.join(
                    get_package_share_directory("amr_bringup"),
                    "urdf",
                    "turtlebot3_burger.urdf.xacro",
                ),
                description="AMR-owned robot description xacro.",
            ),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="",
                description="Optional frame prefix for the robot description.",
            ),
            DeclareLaunchArgument(
                "use_base_driver",
                default_value="true",
                description="Start the AMR-owned TB3 base driver.",
            ),
            DeclareLaunchArgument(
                "use_lidar_driver",
                default_value="true",
                description="Start the AMR-owned TB3 LiDAR driver.",
            ),
            DeclareLaunchArgument(
                "lidar_backend",
                default_value="external_hlds",
                description="LiDAR backend selector: external_hlds or internal.",
            ),
            DeclareLaunchArgument(
                "use_robot_state_publisher",
                default_value="true",
                description="Start robot_state_publisher with the AMR-owned description.",
            ),
            DeclareLaunchArgument(
                "base_port",
                default_value="/dev/ttyACM0",
                description="OpenCR serial port path.",
            ),
            DeclareLaunchArgument(
                "base_baudrate",
                default_value="115200",
                description="OpenCR serial baudrate.",
            ),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="auto",
                description="LiDAR serial port path or 'auto' to resolve a CP210x-backed device.",
            ),
            DeclareLaunchArgument(
                "lidar_baudrate",
                default_value="230400",
                description="LiDAR serial baudrate.",
            ),
            DeclareLaunchArgument(
                "sensor_model",
                default_value="auto",
                description="LiDAR sensor model selector.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Whether the base driver publishes odom to base_footprint TF.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Velocity command topic.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Odometry topic.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu",
                description="IMU topic.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan topic.",
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="Joint states topic.",
            ),
            robot_state_publisher,
            base_driver,
            lidar_driver,
            external_lidar_driver,
        ]
    )
