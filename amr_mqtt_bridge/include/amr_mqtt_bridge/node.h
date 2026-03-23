#ifndef AMR_MQTT_BRIDGE__NODE_H_
#define AMR_MQTT_BRIDGE__NODE_H_

#include <stdbool.h>
#include <stddef.h>

#include <MQTTClient.h>

#include <rcl/client.h>
#include <rcl/publisher.h>
#include <rcl/rcl.h>
#include <rcl/subscription.h>
#include <rcl/wait.h>
#include <rcl_action/action_client.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
#include <tf2_msgs/msg/tf_message.h>
#include <action_msgs/msg/goal_status_array.h>
#include <amr_msgs/action/navigate_to_pose.h>
#include <amr_msgs/msg/motion_status.h>
#include <amr_msgs/msg/obstacle_report.h>
#include <amr_msgs/srv/plan_route.h>
#include <amr_msgs/srv/plan_segment.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#define AMR_MQTT_BRIDGE_MAX_STRING_LENGTH 512
#define AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS 11

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
  char telemetry_robot_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_global_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_local_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_global_costmap[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_local_costmap[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_motion_status[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char telemetry_obstacle_report[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_robot_cmd_vel[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char robot_telemetry_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char robot_telemetry_scan[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char robot_telemetry_odom[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char robot_telemetry_imu[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char robot_telemetry_tf[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char robot_telemetry_tf_static[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char robot_telemetry_joint_states[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char robot_telemetry_robot_description[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_set_initial_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char status_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_set_initial_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char feedback_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char request_plan_segment[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char request_plan_route[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_plan_segment[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char response_plan_route[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
} amr_mqtt_bridge_mqtt_topics_t;

typedef struct amr_mqtt_bridge_ros_interfaces_s
{
  char topic_robot_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_global_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_local_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_map[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_global_costmap[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_local_costmap[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_motion_status[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_obstacle_report[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_velocity[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_scan[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_odom[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_imu[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_tf[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_tf_static[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_joint_states[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_robot_description[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_rviz_goal[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_initial_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_navigate_feedback[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char topic_navigate_status[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char service_plan_segment[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char service_plan_route[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char action_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
} amr_mqtt_bridge_ros_interfaces_t;

typedef struct amr_mqtt_bridge_config_s
{
  amr_mqtt_bridge_broker_config_t broker;
  amr_mqtt_bridge_mqtt_topics_t mqtt;
  amr_mqtt_bridge_ros_interfaces_t ros;
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
  bool raw_passthrough;
  amr_mqtt_bridge_serializer_fn_t serializer;
} amr_mqtt_bridge_telemetry_endpoint_t;

typedef struct amr_mqtt_bridge_navigate_state_s
{
  bool active;
  bool goal_response_pending;
  bool result_response_pending;
  int64_t goal_request_sequence_number;
  int64_t result_request_sequence_number;
  uint8_t goal_uuid[16];
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
} amr_mqtt_bridge_navigate_state_t;

typedef struct amr_mqtt_bridge_ros_state_s
{
  geometry_msgs__msg__PoseStamped robot_pose_message;
  nav_msgs__msg__Path global_path_message;
  nav_msgs__msg__Path local_path_message;
  nav_msgs__msg__OccupancyGrid map_message;
  nav_msgs__msg__OccupancyGrid global_costmap_message;
  nav_msgs__msg__OccupancyGrid local_costmap_message;
  amr_msgs__msg__MotionStatus motion_status_message;
  amr_msgs__msg__ObstacleReport obstacle_report_message;
  geometry_msgs__msg__Twist velocity_message;
  geometry_msgs__msg__PoseWithCovarianceStamped initial_pose_message;
  geometry_msgs__msg__PoseStamped rviz_goal_message;

  rcl_subscription_t robot_pose_subscription;
  rcl_subscription_t global_path_subscription;
  rcl_subscription_t local_path_subscription;
  rcl_subscription_t map_subscription;
  rcl_subscription_t global_costmap_subscription;
  rcl_subscription_t local_costmap_subscription;
  rcl_subscription_t motion_status_subscription;
  rcl_subscription_t obstacle_report_subscription;
  rcl_subscription_t velocity_subscription;
  rcl_subscription_t initial_pose_subscription;
  rcl_subscription_t rviz_goal_subscription;
  rcl_publisher_t robot_pose_publisher;
  rcl_publisher_t global_path_publisher;
  rcl_publisher_t local_path_publisher;
  rcl_publisher_t global_costmap_publisher;
  rcl_publisher_t local_costmap_publisher;
  rcl_publisher_t motion_status_publisher;
  rcl_publisher_t obstacle_report_publisher;
  rcl_publisher_t robot_map_publisher;
  rcl_publisher_t robot_scan_publisher;
  rcl_publisher_t robot_odom_publisher;
  rcl_publisher_t robot_imu_publisher;
  rcl_publisher_t robot_tf_publisher;
  rcl_publisher_t robot_tf_static_publisher;
  rcl_publisher_t robot_joint_states_publisher;
  rcl_publisher_t robot_description_publisher;
  rcl_publisher_t initial_pose_publisher;
  rcl_publisher_t navigate_feedback_publisher;
  rcl_publisher_t navigate_status_publisher;
  rcl_client_t plan_segment_client;
  rcl_client_t plan_route_client;
  rcl_action_client_t navigate_to_pose_client;
  rcl_wait_set_t navigate_wait_set;
  action_msgs__msg__GoalStatusArray navigate_status_message;
  amr_mqtt_bridge_navigate_state_t navigate_state;

  amr_mqtt_bridge_telemetry_endpoint_t telemetry_endpoints[AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS];
  size_t telemetry_endpoint_count;
  bool messages_initialized;
  bool subscriptions_initialized;
  bool robot_pose_publisher_initialized;
  bool global_path_publisher_initialized;
  bool local_path_publisher_initialized;
  bool global_costmap_publisher_initialized;
  bool local_costmap_publisher_initialized;
  bool motion_status_publisher_initialized;
  bool obstacle_report_publisher_initialized;
  bool robot_map_publisher_initialized;
  bool robot_scan_publisher_initialized;
  bool robot_odom_publisher_initialized;
  bool robot_imu_publisher_initialized;
  bool robot_tf_publisher_initialized;
  bool robot_tf_static_publisher_initialized;
  bool robot_joint_states_publisher_initialized;
  bool robot_description_publisher_initialized;
  bool initial_pose_publisher_initialized;
  bool navigate_feedback_publisher_initialized;
  bool navigate_status_publisher_initialized;
  bool plan_segment_client_initialized;
  bool plan_route_client_initialized;
  bool navigate_to_pose_client_initialized;
  bool navigate_wait_set_initialized;
  bool navigate_status_message_initialized;
} amr_mqtt_bridge_ros_state_t;

extern amr_mqtt_bridge_runtime_t g_amr_mqtt_bridge_runtime;
extern amr_mqtt_bridge_mqtt_state_t g_amr_mqtt_bridge_mqtt;
extern amr_mqtt_bridge_config_t g_amr_mqtt_bridge_config;
extern amr_mqtt_bridge_ros_state_t g_amr_mqtt_bridge_ros_state;

int amr_mqtt_bridge_init(int argc, const char * const * argv);
void amr_mqtt_bridge_spin(void);
int amr_mqtt_bridge_shutdown(void);

#endif  // AMR_MQTT_BRIDGE__NODE_H_
