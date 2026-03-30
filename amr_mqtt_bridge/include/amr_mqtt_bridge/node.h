#ifndef AMR_MQTT_BRIDGE__NODE_H_
#define AMR_MQTT_BRIDGE__NODE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <MQTTClient.h>

#include <rcl/arguments.h>
#include <rcl/client.h>
#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcl/rcl.h>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rcl/wait.h>
#include <rcl_action/action_client.h>
#include <rcl_action/wait.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/error_handling.h>
#include <rcutils/logging.h>
#include <rcutils/logging_macros.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/qos_profiles.h>

#include <action_msgs/msg/goal_status_array.h>
#include <action_msgs/msg/detail/goal_status_array__functions.h>
#include <action_msgs/srv/detail/cancel_goal__functions.h>
#include <amr_msgs/action/detail/navigate_to_pose__functions.h>
#include <amr_msgs/action/detail/navigate_to_pose__type_support.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/detail/pose_stamped__functions.h>
#include <geometry_msgs/msg/detail/pose_stamped__type_support.h>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__functions.h>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__type_support.h>
#include <geometry_msgs/msg/detail/twist__functions.h>
#include <geometry_msgs/msg/detail/twist__type_support.h>
#include <amr_msgs/action/navigate_to_pose.h>
#include <amr_msgs/msg/detail/motion_status__functions.h>
#include <amr_msgs/msg/detail/motion_status__type_support.h>
#include <amr_msgs/srv/detail/plan_route__functions.h>
#include <amr_msgs/srv/detail/plan_segment__functions.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <nav_msgs/msg/detail/occupancy_grid__functions.h>
#include <nav_msgs/msg/detail/occupancy_grid__type_support.h>
#include <nav_msgs/msg/detail/odometry__functions.h>
#include <nav_msgs/msg/detail/odometry__type_support.h>
#include <nav_msgs/msg/detail/path__functions.h>
#include <nav_msgs/msg/detail/path__type_support.h>
#include <amr_msgs/srv/plan_route.h>
#include <amr_msgs/srv/plan_segment.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rosidl_runtime_c/action_type_support_struct.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/detail/battery_state__functions.h>
#include <sensor_msgs/msg/detail/battery_state__type_support.h>
#include <sensor_msgs/msg/detail/imu__functions.h>
#include <sensor_msgs/msg/detail/imu__type_support.h>
#include <sensor_msgs/msg/detail/joint_state__functions.h>
#include <sensor_msgs/msg/detail/joint_state__type_support.h>
#include <sensor_msgs/msg/detail/laser_scan__functions.h>
#include <sensor_msgs/msg/detail/laser_scan__type_support.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/detail/string__functions.h>
#include <std_msgs/msg/detail/string__type_support.h>
#include <tf2_msgs/msg/tf_message.h>
#include <tf2_msgs/msg/detail/tf_message__functions.h>
#include <tf2_msgs/msg/detail/tf_message__type_support.h>
#include <amr_msgs/msg/motion_status.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#define AMR_MQTT_BRIDGE_NODE_NAME "mqtt_bridge"
#define AMR_MQTT_BRIDGE_NODE_NAMESPACE "/amr"
#define AMR_MQTT_BRIDGE_MAX_STRING_LENGTH 512
#define AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS 18
#define AMR_MQTT_BRIDGE_MAX_FOOTPRINT_POLYGON_VALUES 32

typedef char * (* amr_mqtt_bridge_serializer_fn_t)(const void * message);

typedef struct amr_mqtt_bridge_runtime_s
{
  rcl_allocator_t allocator;
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;
  int return_code;
  bool is_initialized;
} amr_mqtt_bridge_runtime_t;

typedef struct amr_mqtt_bridge_mqtt_state_s
{
  MQTTClient client;
  char broker_uri[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  bool client_created;
  bool connected;
  bool command_subscriptions_registered;
  long last_reconnect_attempt_sec;
} amr_mqtt_bridge_mqtt_state_t;

typedef struct amr_mqtt_bridge_broker_config_s
{
  char host[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  int port;
  char client_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  int keep_alive_sec;
  bool clean_session;
  char username[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char password[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
} amr_mqtt_bridge_broker_config_t;

typedef struct amr_mqtt_bridge_mqtt_topics_s
{
  char root[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  int telemetry_qos;
  int command_qos;
  int service_qos;
  char telemetry_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_robot_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_global_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_local_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_global_costmap[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_local_costmap[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_motion_status[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_scan[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_odom[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_imu[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_tf[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_tf_static[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_joint_states[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_robot_description[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_battery_state[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_temp_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_mapping_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_slam_graph[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_cmd_vel[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_save_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_set_initial_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_cancel_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_ping[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char feedback_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char status_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_set_initial_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_save_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_ping[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char request_plan_segment[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char request_plan_route[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_plan_segment[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_plan_route[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
} amr_mqtt_bridge_mqtt_topics_t;

typedef struct amr_mqtt_bridge_ros_interfaces_s
{
  char topic_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_global_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_local_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_global_costmap[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_local_costmap[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_motion_status[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_scan[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_odom[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_imu[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_tf[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_tf_static[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_joint_states[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_description[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_battery_state[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_temp_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_mapping_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_slam_graph[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_cmd_vel[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_initial_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_navigate_feedback[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_navigate_status[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char service_plan_segment[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char service_plan_route[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char action_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char save_directory[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
} amr_mqtt_bridge_ros_interfaces_t;

typedef struct amr_mqtt_bridge_config_s
{
  amr_mqtt_bridge_broker_config_t broker;
  amr_mqtt_bridge_mqtt_topics_t mqtt;
  amr_mqtt_bridge_ros_interfaces_t ros;
  double footprint_polygon[AMR_MQTT_BRIDGE_MAX_FOOTPRINT_POLYGON_VALUES];
  size_t footprint_polygon_size;
} amr_mqtt_bridge_config_t;

typedef struct amr_mqtt_bridge_telemetry_endpoint_s
{
  const char * label;
  const char * ros_topic;
  const char * mqtt_topic;
  rcl_subscription_t * subscription;
  void * message;
  const rosidl_message_type_support_t * type_support;
  const rmw_qos_profile_t * qos_profile;
  int mqtt_qos;
  bool retained;
  bool raw_passthrough;
  amr_mqtt_bridge_serializer_fn_t serializer;
  bool raw_publish_once;
  bool viz_publish_once;
  uint64_t raw_min_period_ms;
  uint64_t viz_min_period_ms;
  uint64_t last_raw_publish_ms;
  uint64_t last_viz_publish_ms;
  bool raw_published_once;
  bool viz_published_once;
} amr_mqtt_bridge_telemetry_endpoint_t;

typedef struct amr_mqtt_bridge_ros_state_s
{
  nav_msgs__msg__OccupancyGrid map_message;
  geometry_msgs__msg__PoseStamped robot_pose_message;
  nav_msgs__msg__Path global_path_message;
  nav_msgs__msg__Path local_path_message;
  nav_msgs__msg__OccupancyGrid global_costmap_message;
  nav_msgs__msg__OccupancyGrid local_costmap_message;
  amr_msgs__msg__MotionStatus motion_status_message;
  sensor_msgs__msg__LaserScan scan_message;
  nav_msgs__msg__Odometry odom_message;
  sensor_msgs__msg__Imu imu_message;
  tf2_msgs__msg__TFMessage tf_message;
  tf2_msgs__msg__TFMessage tf_static_message;
  sensor_msgs__msg__JointState joint_states_message;
  std_msgs__msg__String robot_description_message;
  sensor_msgs__msg__BatteryState battery_state_message;
  nav_msgs__msg__OccupancyGrid temp_map_message;
  geometry_msgs__msg__PoseStamped mapping_pose_message;
  std_msgs__msg__String slam_graph_message;
  geometry_msgs__msg__Twist cmd_vel_message;
  action_msgs__msg__GoalStatusArray navigate_status_message;

  rcl_subscription_t map_subscription;
  rcl_subscription_t robot_pose_subscription;
  rcl_subscription_t global_path_subscription;
  rcl_subscription_t local_path_subscription;
  rcl_subscription_t global_costmap_subscription;
  rcl_subscription_t local_costmap_subscription;
  rcl_subscription_t motion_status_subscription;
  rcl_subscription_t scan_subscription;
  rcl_subscription_t odom_subscription;
  rcl_subscription_t imu_subscription;
  rcl_subscription_t tf_subscription;
  rcl_subscription_t tf_static_subscription;
  rcl_subscription_t joint_states_subscription;
  rcl_subscription_t robot_description_subscription;
  rcl_subscription_t battery_state_subscription;
  rcl_subscription_t temp_map_subscription;
  rcl_subscription_t mapping_pose_subscription;
  rcl_subscription_t slam_graph_subscription;
  rcl_publisher_t cmd_vel_publisher;
  rcl_publisher_t initial_pose_publisher;
  rcl_publisher_t navigate_feedback_publisher;
  rcl_publisher_t navigate_status_publisher;
  rcl_client_t plan_segment_client;
  rcl_client_t plan_route_client;
  rcl_action_client_t navigate_to_pose_client;
  rcl_wait_set_t navigate_wait_set;

  amr_mqtt_bridge_telemetry_endpoint_t telemetry_endpoints[AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS];
  size_t telemetry_endpoint_count;
  bool messages_initialized;
  bool subscriptions_initialized;
  bool cmd_vel_publisher_initialized;
  bool initial_pose_publisher_initialized;
  bool navigate_feedback_publisher_initialized;
  bool navigate_status_publisher_initialized;
  bool plan_segment_client_initialized;
  bool plan_route_client_initialized;
  bool navigate_to_pose_client_initialized;
  bool navigate_wait_set_initialized;
  bool navigate_status_message_initialized;
  struct
  {
    bool active;
    bool goal_response_pending;
    bool result_response_pending;
    int64_t goal_request_sequence_number;
    int64_t result_request_sequence_number;
    uint8_t goal_uuid[16];
    char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  } navigate_state;
} amr_mqtt_bridge_ros_state_t;

extern amr_mqtt_bridge_runtime_t g_amr_mqtt_bridge_runtime;
extern amr_mqtt_bridge_mqtt_state_t g_amr_mqtt_bridge_mqtt;
extern amr_mqtt_bridge_config_t g_amr_mqtt_bridge_config;
extern amr_mqtt_bridge_ros_state_t g_amr_mqtt_bridge_ros_state;

rcl_ret_t amr_mqtt_bridge_initialize(int argc, const char *argv[]);
void amr_mqtt_bridge_run(void);
rcl_ret_t amr_mqtt_bridge_terminate(void);
int amr_mqtt_bridge_connect_mqtt(void);
void amr_mqtt_bridge_disconnect_mqtt(void);
bool amr_mqtt_bridge_ensure_connected(void);
bool amr_mqtt_bridge_publish_payload(
  const char * mqtt_topic,
  const char * payload,
  int qos,
  bool retained);
bool amr_mqtt_bridge_publish_binary_payload(
  const char * mqtt_topic,
  const void * payload,
  size_t payload_length,
  int qos,
  bool retained);

#endif
