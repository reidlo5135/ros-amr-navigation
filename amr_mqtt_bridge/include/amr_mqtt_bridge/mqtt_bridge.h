#ifndef AMR_MQTT_BRIDGE__MQTT_BRIDGE_H_
#define AMR_MQTT_BRIDGE__MQTT_BRIDGE_H_

#include <stdbool.h>
#include <stddef.h>

#include <MQTTClient.h>

#include <rcl/rcl.h>
#include <rcl/subscription.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <nav_msgs/msg/path.h>
#include <amr_msgs/msg/motion_status.h>
#include <amr_msgs/msg/obstacle_report.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#define AMR_MQTT_BRIDGE_MAX_STRING_LENGTH 512
#define AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS 8

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
  char command_navigate_to_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  char command_set_initial_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
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
  char topic_initial_pose[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
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
  amr_mqtt_bridge_serializer_fn_t serializer;
} amr_mqtt_bridge_telemetry_endpoint_t;

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

  rcl_subscription_t robot_pose_subscription;
  rcl_subscription_t global_path_subscription;
  rcl_subscription_t local_path_subscription;
  rcl_subscription_t map_subscription;
  rcl_subscription_t global_costmap_subscription;
  rcl_subscription_t local_costmap_subscription;
  rcl_subscription_t motion_status_subscription;
  rcl_subscription_t obstacle_report_subscription;

  amr_mqtt_bridge_telemetry_endpoint_t telemetry_endpoints[AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS];
  size_t telemetry_endpoint_count;
  bool messages_initialized;
  bool subscriptions_initialized;
} amr_mqtt_bridge_ros_state_t;

extern amr_mqtt_bridge_runtime_t g_amr_mqtt_bridge_runtime;
extern amr_mqtt_bridge_mqtt_state_t g_amr_mqtt_bridge_mqtt;
extern amr_mqtt_bridge_config_t g_amr_mqtt_bridge_config;
extern amr_mqtt_bridge_ros_state_t g_amr_mqtt_bridge_ros_state;

int amr_mqtt_bridge_init(int argc, const char * const * argv);
void amr_mqtt_bridge_spin(void);
int amr_mqtt_bridge_shutdown(void);

#endif  // AMR_MQTT_BRIDGE__MQTT_BRIDGE_H_
