#include "amr_mqtt_bridge/mqtt_bridge.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rcl/graph.h>
#include <rcl/arguments.h>
#include <rcl/client.h>
#include <rcl/error_handling.h>
#include <rcl/publisher.h>
#include <rcl/wait.h>
#include <rcl_action/action_client.h>
#include <rcl_action/wait.h>
#include <rcutils/error_handling.h>
#include <rcutils/logging.h>
#include <rcutils/logging_macros.h>
#include <rosidl_runtime_c/action_type_support_struct.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rcl_yaml_param_parser/parser.h>

#include <action_msgs/msg/detail/goal_status_array__functions.h>
#include <amr_msgs/action/detail/navigate_to_pose__functions.h>
#include <amr_msgs/action/detail/navigate_to_pose__type_support.h>
#include <amr_msgs/srv/detail/plan_route__functions.h>
#include <amr_msgs/srv/detail/plan_segment__functions.h>
#include <geometry_msgs/msg/detail/pose_stamped__functions.h>
#include <geometry_msgs/msg/detail/pose_stamped__type_support.h>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__functions.h>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__type_support.h>
#include <nav_msgs/msg/detail/occupancy_grid__functions.h>
#include <nav_msgs/msg/detail/occupancy_grid__type_support.h>
#include <nav_msgs/msg/detail/path__functions.h>
#include <nav_msgs/msg/detail/path__type_support.h>
#include <amr_msgs/msg/detail/motion_status__functions.h>
#include <amr_msgs/msg/detail/motion_status__type_support.h>
#include <amr_msgs/msg/detail/obstacle_report__functions.h>
#include <amr_msgs/msg/detail/obstacle_report__type_support.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>

amr_mqtt_bridge_runtime_t g_amr_mqtt_bridge_runtime = {0};

amr_mqtt_bridge_mqtt_state_t g_amr_mqtt_bridge_mqtt = {0};

amr_mqtt_bridge_config_t g_amr_mqtt_bridge_config = {0};

amr_mqtt_bridge_ros_state_t g_amr_mqtt_bridge_ros_state = {0};

typedef struct amr_mqtt_bridge_string_builder_s
{
  char * data;
  size_t length;
  size_t capacity;
} amr_mqtt_bridge_string_builder_t;

static void amr_mqtt_bridge_fini_messages(void);
static void amr_mqtt_bridge_fini_subscriptions(void);
static void amr_mqtt_bridge_log_rcl_fini_error(const char * label, rcl_ret_t rc);
static bool amr_mqtt_bridge_ensure_connected(void);
static int amr_mqtt_bridge_subscribe_command_topics(void);
static void amr_mqtt_bridge_disconnect_mqtt(void);
static void amr_mqtt_bridge_poll_mqtt(void);
static void amr_mqtt_bridge_poll_navigate_action(void);
static int amr_mqtt_bridge_init_command_interfaces(void);
static void amr_mqtt_bridge_fini_command_interfaces(void);

static const char * k_node_name = "/amr/mqtt_bridge";
static const rmw_qos_profile_t k_default_qos = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};
static const rmw_qos_profile_t k_transient_local_qos = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

static void amr_mqtt_bridge_copy_string(char * destination, size_t capacity, const char * source)
{
  if (capacity == 0U) {
    return;
  }

  if (source == NULL) {
    destination[0] = '\0';
    return;
  }

  (void)snprintf(destination, capacity, "%s", source);
}

static void amr_mqtt_bridge_set_default_config(void)
{
  memset(&g_amr_mqtt_bridge_config, 0, sizeof(g_amr_mqtt_bridge_config));

  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.broker.host, sizeof(g_amr_mqtt_bridge_config.broker.host), "127.0.0.1");
  g_amr_mqtt_bridge_config.broker.port = 1883;
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.broker.client_id,
    sizeof(g_amr_mqtt_bridge_config.broker.client_id),
    "amr_mqtt_bridge");
  g_amr_mqtt_bridge_config.broker.keep_alive_sec = 20;
  g_amr_mqtt_bridge_config.broker.clean_session = true;

  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.root, sizeof(g_amr_mqtt_bridge_config.mqtt.root), "amr");
  g_amr_mqtt_bridge_config.mqtt.telemetry_qos = 0;
  g_amr_mqtt_bridge_config.mqtt.command_qos = 1;
  g_amr_mqtt_bridge_config.mqtt.service_qos = 1;
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose), "amr/telemetry/robot_pose");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.telemetry_global_path, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_global_path), "amr/telemetry/global_path");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.telemetry_local_path, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_local_path), "amr/telemetry/local_path");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.telemetry_map, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_map), "amr/telemetry/map");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap), "amr/telemetry/global_costmap");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap), "amr/telemetry/local_costmap");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status), "amr/telemetry/motion_status");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.telemetry_obstacle_report, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_obstacle_report), "amr/telemetry/obstacle_report");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose), "amr/command/navigate_to_pose");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose), "amr/command/set_initial_pose");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose), "amr/feedback/navigate_to_pose");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose), "amr/response/set_initial_pose");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose), "amr/response/navigate_to_pose");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.request_plan_segment, sizeof(g_amr_mqtt_bridge_config.mqtt.request_plan_segment), "amr/request/plan_segment");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.request_plan_route, sizeof(g_amr_mqtt_bridge_config.mqtt.request_plan_route), "amr/request/plan_route");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.response_plan_segment, sizeof(g_amr_mqtt_bridge_config.mqtt.response_plan_segment), "amr/response/plan_segment");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.mqtt.response_plan_route, sizeof(g_amr_mqtt_bridge_config.mqtt.response_plan_route), "amr/response/plan_route");

  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_robot_pose, sizeof(g_amr_mqtt_bridge_config.ros.topic_robot_pose), "/amr/localization/pose");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_global_path, sizeof(g_amr_mqtt_bridge_config.ros.topic_global_path), "/amr/planner/global");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_local_path, sizeof(g_amr_mqtt_bridge_config.ros.topic_local_path), "/amr/planner/local");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_map, sizeof(g_amr_mqtt_bridge_config.ros.topic_map), "/amr/map/data");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_global_costmap, sizeof(g_amr_mqtt_bridge_config.ros.topic_global_costmap), "/amr/costmap/global");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_local_costmap, sizeof(g_amr_mqtt_bridge_config.ros.topic_local_costmap), "/amr/costmap/local");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_motion_status, sizeof(g_amr_mqtt_bridge_config.ros.topic_motion_status), "/amr/motion/status");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_obstacle_report, sizeof(g_amr_mqtt_bridge_config.ros.topic_obstacle_report), "/amr/obstacle/report");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.topic_initial_pose, sizeof(g_amr_mqtt_bridge_config.ros.topic_initial_pose), "/amr/localization/initial_pose");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.service_plan_segment, sizeof(g_amr_mqtt_bridge_config.ros.service_plan_segment), "/amr/global_planner/plan_segment");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.service_plan_route, sizeof(g_amr_mqtt_bridge_config.ros.service_plan_route), "/amr/global_planner/plan_route");
  amr_mqtt_bridge_copy_string(g_amr_mqtt_bridge_config.ros.action_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.ros.action_navigate_to_pose), "/amr/navigator/navigate_to_pose");
}

static const rcl_node_params_t * amr_mqtt_bridge_find_node_params(
  const rcl_params_t * params,
  const char * node_name)
{
  if (params == NULL || node_name == NULL) {
    return NULL;
  }

  for (size_t node_index = 0; node_index < params->num_nodes; ++node_index) {
    const char * current_name = params->node_names[node_index];
    if (current_name == NULL) {
      continue;
    }

    if (strcmp(current_name, node_name) == 0) {
      return &params->params[node_index];
    }

    if (node_name[0] == '/' && strcmp(current_name, node_name + 1) == 0) {
      return &params->params[node_index];
    }

    if (current_name[0] == '/' && strcmp(current_name + 1, node_name) == 0) {
      return &params->params[node_index];
    }
  }

  return NULL;
}

static const rcl_variant_t * amr_mqtt_bridge_find_param_variant(
  const rcl_node_params_t * node_params,
  const char * parameter_name)
{
  if (node_params == NULL || parameter_name == NULL) {
    return NULL;
  }

  for (size_t param_index = 0; param_index < node_params->num_params; ++param_index) {
    const char * current_name = node_params->parameter_names[param_index];
    if (current_name != NULL && strcmp(current_name, parameter_name) == 0) {
      return &node_params->parameter_values[param_index];
    }
  }

  return NULL;
}

static void amr_mqtt_bridge_read_string_param(
  const rcl_node_params_t * node_params,
  const char * parameter_name,
  char * destination,
  size_t capacity)
{
  const rcl_variant_t * variant = amr_mqtt_bridge_find_param_variant(node_params, parameter_name);
  if (variant == NULL || variant->string_value == NULL) {
    return;
  }

  amr_mqtt_bridge_copy_string(destination, capacity, variant->string_value);
}

static void amr_mqtt_bridge_read_integer_param(
  const rcl_node_params_t * node_params,
  const char * parameter_name,
  int * destination)
{
  const rcl_variant_t * variant = amr_mqtt_bridge_find_param_variant(node_params, parameter_name);
  if (variant == NULL || variant->integer_value == NULL || destination == NULL) {
    return;
  }

  *destination = (int)(*variant->integer_value);
}

static void amr_mqtt_bridge_read_bool_param(
  const rcl_node_params_t * node_params,
  const char * parameter_name,
  bool * destination)
{
  const rcl_variant_t * variant = amr_mqtt_bridge_find_param_variant(node_params, parameter_name);
  if (variant == NULL || variant->bool_value == NULL || destination == NULL) {
    return;
  }

  *destination = *variant->bool_value;
}

static void amr_mqtt_bridge_load_parameter_overrides(void)
{
  rcl_params_t * parameter_overrides = NULL;
  rcl_ret_t rc = rcl_arguments_get_param_overrides(
    &g_amr_mqtt_bridge_runtime.support.context.global_arguments,
    &parameter_overrides);
  if (rc != RCL_RET_OK) {
    RCUTILS_LOG_WARN_NAMED(
      "amr_mqtt_bridge",
      "Failed to read parameter overrides, using defaults: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
    return;
  }

  if (parameter_overrides == NULL) {
    return;
  }

  const rcl_node_params_t * node_params =
    amr_mqtt_bridge_find_node_params(parameter_overrides, k_node_name);
  if (node_params != NULL) {
    amr_mqtt_bridge_read_string_param(node_params, "broker.host", g_amr_mqtt_bridge_config.broker.host, sizeof(g_amr_mqtt_bridge_config.broker.host));
    amr_mqtt_bridge_read_integer_param(node_params, "broker.port", &g_amr_mqtt_bridge_config.broker.port);
    amr_mqtt_bridge_read_string_param(node_params, "broker.client_id", g_amr_mqtt_bridge_config.broker.client_id, sizeof(g_amr_mqtt_bridge_config.broker.client_id));
    amr_mqtt_bridge_read_integer_param(node_params, "broker.keep_alive_sec", &g_amr_mqtt_bridge_config.broker.keep_alive_sec);
    amr_mqtt_bridge_read_bool_param(node_params, "broker.clean_session", &g_amr_mqtt_bridge_config.broker.clean_session);
    amr_mqtt_bridge_read_string_param(node_params, "broker.username", g_amr_mqtt_bridge_config.broker.username, sizeof(g_amr_mqtt_bridge_config.broker.username));
    amr_mqtt_bridge_read_string_param(node_params, "broker.password", g_amr_mqtt_bridge_config.broker.password, sizeof(g_amr_mqtt_bridge_config.broker.password));

    amr_mqtt_bridge_read_string_param(node_params, "mqtt.root", g_amr_mqtt_bridge_config.mqtt.root, sizeof(g_amr_mqtt_bridge_config.mqtt.root));
    amr_mqtt_bridge_read_integer_param(node_params, "mqtt.qos.telemetry", &g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
    amr_mqtt_bridge_read_integer_param(node_params, "mqtt.qos.command", &g_amr_mqtt_bridge_config.mqtt.command_qos);
    amr_mqtt_bridge_read_integer_param(node_params, "mqtt.qos.service", &g_amr_mqtt_bridge_config.mqtt.service_qos);
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.robot_pose", g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.global_path", g_amr_mqtt_bridge_config.mqtt.telemetry_global_path, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_global_path));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.local_path", g_amr_mqtt_bridge_config.mqtt.telemetry_local_path, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_local_path));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.map", g_amr_mqtt_bridge_config.mqtt.telemetry_map, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_map));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.global_costmap", g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.local_costmap", g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.motion_status", g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.obstacle_report", g_amr_mqtt_bridge_config.mqtt.telemetry_obstacle_report, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_obstacle_report));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.command.navigate_to_pose", g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.command.set_initial_pose", g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.feedback.navigate_to_pose", g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.set_initial_pose", g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.navigate_to_pose", g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.request.plan_segment", g_amr_mqtt_bridge_config.mqtt.request_plan_segment, sizeof(g_amr_mqtt_bridge_config.mqtt.request_plan_segment));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.request.plan_route", g_amr_mqtt_bridge_config.mqtt.request_plan_route, sizeof(g_amr_mqtt_bridge_config.mqtt.request_plan_route));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.plan_segment", g_amr_mqtt_bridge_config.mqtt.response_plan_segment, sizeof(g_amr_mqtt_bridge_config.mqtt.response_plan_segment));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.plan_route", g_amr_mqtt_bridge_config.mqtt.response_plan_route, sizeof(g_amr_mqtt_bridge_config.mqtt.response_plan_route));

    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.robot_pose", g_amr_mqtt_bridge_config.ros.topic_robot_pose, sizeof(g_amr_mqtt_bridge_config.ros.topic_robot_pose));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.global_path", g_amr_mqtt_bridge_config.ros.topic_global_path, sizeof(g_amr_mqtt_bridge_config.ros.topic_global_path));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.local_path", g_amr_mqtt_bridge_config.ros.topic_local_path, sizeof(g_amr_mqtt_bridge_config.ros.topic_local_path));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.map", g_amr_mqtt_bridge_config.ros.topic_map, sizeof(g_amr_mqtt_bridge_config.ros.topic_map));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.global_costmap", g_amr_mqtt_bridge_config.ros.topic_global_costmap, sizeof(g_amr_mqtt_bridge_config.ros.topic_global_costmap));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.local_costmap", g_amr_mqtt_bridge_config.ros.topic_local_costmap, sizeof(g_amr_mqtt_bridge_config.ros.topic_local_costmap));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.motion_status", g_amr_mqtt_bridge_config.ros.topic_motion_status, sizeof(g_amr_mqtt_bridge_config.ros.topic_motion_status));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.obstacle_report", g_amr_mqtt_bridge_config.ros.topic_obstacle_report, sizeof(g_amr_mqtt_bridge_config.ros.topic_obstacle_report));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.initial_pose", g_amr_mqtt_bridge_config.ros.topic_initial_pose, sizeof(g_amr_mqtt_bridge_config.ros.topic_initial_pose));
    amr_mqtt_bridge_read_string_param(node_params, "ros.services.plan_segment", g_amr_mqtt_bridge_config.ros.service_plan_segment, sizeof(g_amr_mqtt_bridge_config.ros.service_plan_segment));
    amr_mqtt_bridge_read_string_param(node_params, "ros.services.plan_route", g_amr_mqtt_bridge_config.ros.service_plan_route, sizeof(g_amr_mqtt_bridge_config.ros.service_plan_route));
    amr_mqtt_bridge_read_string_param(node_params, "ros.actions.navigate_to_pose", g_amr_mqtt_bridge_config.ros.action_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.ros.action_navigate_to_pose));
  }

  rcl_yaml_node_struct_fini(parameter_overrides);
}

static void amr_mqtt_bridge_log_config(void)
{
  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_bridge",
    "Configured broker %s:%d with client_id='%s'",
    g_amr_mqtt_bridge_config.broker.host,
    g_amr_mqtt_bridge_config.broker.port,
    g_amr_mqtt_bridge_config.broker.client_id);
}

static int amr_mqtt_bridge_subscribe_command_topics(void)
{
  int mqtt_rc = 0;

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose,
    g_amr_mqtt_bridge_config.mqtt.command_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose,
    g_amr_mqtt_bridge_config.mqtt.command_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.request_plan_segment,
    g_amr_mqtt_bridge_config.mqtt.service_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.request_plan_route,
    g_amr_mqtt_bridge_config.mqtt.service_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = true;
  return MQTTCLIENT_SUCCESS;
}

static void amr_mqtt_bridge_log_rcl_fini_error(const char * label, rcl_ret_t rc)
{
  if (rc == RCL_RET_OK) {
    return;
  }

  RCUTILS_LOG_ERROR_NAMED(
    "amr_mqtt_bridge",
    "Failed to finalize %s: %s",
    label,
    rcl_get_error_string().str);
  rcl_reset_error();
  g_amr_mqtt_bridge_runtime.return_code = 1;
}

static int amr_mqtt_bridge_connect_mqtt(void)
{
  int mqtt_rc = 0;
  int broker_uri_length = 0;
  MQTTClient_connectOptions connect_options = MQTTClient_connectOptions_initializer;

  broker_uri_length = snprintf(
    g_amr_mqtt_bridge_mqtt.broker_uri,
    sizeof(g_amr_mqtt_bridge_mqtt.broker_uri),
    "tcp://%s:%d",
    g_amr_mqtt_bridge_config.broker.host,
    g_amr_mqtt_bridge_config.broker.port);
  if (broker_uri_length < 0 ||
    (size_t)broker_uri_length >= sizeof(g_amr_mqtt_bridge_mqtt.broker_uri))
  {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Broker URI is too long for configured host '%s'",
      g_amr_mqtt_bridge_config.broker.host);
    return 1;
  }

  mqtt_rc = MQTTClient_create(
    &g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_mqtt.broker_uri,
    g_amr_mqtt_bridge_config.broker.client_id,
    MQTTCLIENT_PERSISTENCE_NONE,
    NULL);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Failed to create MQTT client for %s: rc=%d",
      g_amr_mqtt_bridge_mqtt.broker_uri,
      mqtt_rc);
    return 1;
  }

  g_amr_mqtt_bridge_mqtt.client_created = true;
  connect_options.keepAliveInterval = g_amr_mqtt_bridge_config.broker.keep_alive_sec;
  connect_options.cleansession = g_amr_mqtt_bridge_config.broker.clean_session ? 1 : 0;
  if (g_amr_mqtt_bridge_config.broker.username[0] != '\0') {
    connect_options.username = g_amr_mqtt_bridge_config.broker.username;
  }
  if (g_amr_mqtt_bridge_config.broker.password[0] != '\0') {
    connect_options.password = g_amr_mqtt_bridge_config.broker.password;
  }

  mqtt_rc = MQTTClient_connect(g_amr_mqtt_bridge_mqtt.client, &connect_options);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Failed to connect to MQTT broker %s: rc=%d (%s)",
      g_amr_mqtt_bridge_mqtt.broker_uri,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
    MQTTClient_destroy(&g_amr_mqtt_bridge_mqtt.client);
    g_amr_mqtt_bridge_mqtt.client_created = false;
    return 1;
  }

  g_amr_mqtt_bridge_mqtt.connected = true;
  g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = false;
  g_amr_mqtt_bridge_mqtt.last_reconnect_attempt_sec = 0;
  mqtt_rc = amr_mqtt_bridge_subscribe_command_topics();
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Failed to subscribe MQTT command topics: rc=%d (%s)",
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
    amr_mqtt_bridge_disconnect_mqtt();
    return 1;
  }
  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_bridge",
    "Connected to MQTT broker at %s",
    g_amr_mqtt_bridge_mqtt.broker_uri);
  return 0;
}

static void amr_mqtt_bridge_disconnect_mqtt(void)
{
  if (g_amr_mqtt_bridge_mqtt.connected) {
    (void)MQTTClient_disconnect(g_amr_mqtt_bridge_mqtt.client, 1000);
    g_amr_mqtt_bridge_mqtt.connected = false;
  }
  g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = false;

  if (g_amr_mqtt_bridge_mqtt.client_created) {
    MQTTClient_destroy(&g_amr_mqtt_bridge_mqtt.client);
    g_amr_mqtt_bridge_mqtt.client_created = false;
  }
}

static bool amr_mqtt_bridge_ensure_connected(void)
{
  long now_sec = (long)time(NULL);
  int mqtt_rc = 0;
  MQTTClient_connectOptions connect_options = MQTTClient_connectOptions_initializer;

  if (!g_amr_mqtt_bridge_mqtt.client_created) {
    return false;
  }

  if (MQTTClient_isConnected(g_amr_mqtt_bridge_mqtt.client)) {
    g_amr_mqtt_bridge_mqtt.connected = true;
    return true;
  }

  g_amr_mqtt_bridge_mqtt.connected = false;
  if (now_sec == g_amr_mqtt_bridge_mqtt.last_reconnect_attempt_sec) {
    return false;
  }

  g_amr_mqtt_bridge_mqtt.last_reconnect_attempt_sec = now_sec;
  connect_options.keepAliveInterval = g_amr_mqtt_bridge_config.broker.keep_alive_sec;
  connect_options.cleansession = g_amr_mqtt_bridge_config.broker.clean_session ? 1 : 0;
  if (g_amr_mqtt_bridge_config.broker.username[0] != '\0') {
    connect_options.username = g_amr_mqtt_bridge_config.broker.username;
  }
  if (g_amr_mqtt_bridge_config.broker.password[0] != '\0') {
    connect_options.password = g_amr_mqtt_bridge_config.broker.password;
  }

  mqtt_rc = MQTTClient_connect(g_amr_mqtt_bridge_mqtt.client, &connect_options);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    RCUTILS_LOG_WARN_NAMED(
      "amr_mqtt_bridge",
      "Failed to reconnect MQTT client to %s: rc=%d (%s)",
      g_amr_mqtt_bridge_mqtt.broker_uri,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
    return false;
  }

  g_amr_mqtt_bridge_mqtt.connected = true;
  if (!g_amr_mqtt_bridge_mqtt.command_subscriptions_registered) {
    mqtt_rc = amr_mqtt_bridge_subscribe_command_topics();
    if (mqtt_rc != MQTTCLIENT_SUCCESS) {
      RCUTILS_LOG_WARN_NAMED(
        "amr_mqtt_bridge",
        "Failed to resubscribe MQTT command topics on reconnect: rc=%d (%s)",
        mqtt_rc,
        MQTTClient_strerror(mqtt_rc));
      g_amr_mqtt_bridge_mqtt.connected = false;
      (void)MQTTClient_disconnect(g_amr_mqtt_bridge_mqtt.client, 1000);
      return false;
    }
  }
  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_bridge",
    "Reconnected to MQTT broker at %s",
    g_amr_mqtt_bridge_mqtt.broker_uri);
  return true;
}

static bool amr_mqtt_bridge_builder_init(
  amr_mqtt_bridge_string_builder_t * builder,
  size_t initial_capacity)
{
  builder->data = (char *)malloc(initial_capacity);
  if (builder->data == NULL) {
    builder->length = 0U;
    builder->capacity = 0U;
    return false;
  }

  builder->data[0] = '\0';
  builder->length = 0U;
  builder->capacity = initial_capacity;
  return true;
}

static void amr_mqtt_bridge_builder_fini(amr_mqtt_bridge_string_builder_t * builder)
{
  if (builder->data != NULL) {
    free(builder->data);
  }
  builder->data = NULL;
  builder->length = 0U;
  builder->capacity = 0U;
}

static bool amr_mqtt_bridge_builder_reserve(
  amr_mqtt_bridge_string_builder_t * builder,
  size_t additional_length)
{
  size_t required_capacity = builder->length + additional_length + 1U;
  if (required_capacity <= builder->capacity) {
    return true;
  }

  size_t new_capacity = builder->capacity == 0U ? 256U : builder->capacity;
  while (new_capacity < required_capacity) {
    new_capacity *= 2U;
  }

  char * resized = (char *)realloc(builder->data, new_capacity);
  if (resized == NULL) {
    return false;
  }

  builder->data = resized;
  builder->capacity = new_capacity;
  return true;
}

static bool amr_mqtt_bridge_builder_append(
  amr_mqtt_bridge_string_builder_t * builder,
  const char * text)
{
  size_t text_length = strlen(text);
  if (!amr_mqtt_bridge_builder_reserve(builder, text_length)) {
    return false;
  }

  memcpy(builder->data + builder->length, text, text_length + 1U);
  builder->length += text_length;
  return true;
}

static bool amr_mqtt_bridge_builder_appendf(
  amr_mqtt_bridge_string_builder_t * builder,
  const char * format,
  ...)
{
  va_list args;
  va_list args_copy;
  va_start(args, format);
  va_copy(args_copy, args);
  int required = vsnprintf(NULL, 0, format, args_copy);
  va_end(args_copy);
  if (required < 0) {
    va_end(args);
    return false;
  }

  if (!amr_mqtt_bridge_builder_reserve(builder, (size_t)required)) {
    va_end(args);
    return false;
  }

  (void)vsnprintf(builder->data + builder->length, builder->capacity - builder->length, format, args);
  builder->length += (size_t)required;
  va_end(args);
  return true;
}

static bool amr_mqtt_bridge_builder_append_json_string(
  amr_mqtt_bridge_string_builder_t * builder,
  const char * text)
{
  const char * safe_text = text != NULL ? text : "";
  if (!amr_mqtt_bridge_builder_append(builder, "\"")) {
    return false;
  }

  for (size_t index = 0; safe_text[index] != '\0'; ++index) {
    char current = safe_text[index];
    switch (current) {
      case '\\':
        if (!amr_mqtt_bridge_builder_append(builder, "\\\\")) {
          return false;
        }
        break;
      case '"':
        if (!amr_mqtt_bridge_builder_append(builder, "\\\"")) {
          return false;
        }
        break;
      case '\n':
        if (!amr_mqtt_bridge_builder_append(builder, "\\n")) {
          return false;
        }
        break;
      case '\r':
        if (!amr_mqtt_bridge_builder_append(builder, "\\r")) {
          return false;
        }
        break;
      case '\t':
        if (!amr_mqtt_bridge_builder_append(builder, "\\t")) {
          return false;
        }
        break;
      default:
      {
        char buffer[2] = {current, '\0'};
        if (!amr_mqtt_bridge_builder_append(builder, buffer)) {
          return false;
        }
        break;
      }
    }
  }

  return amr_mqtt_bridge_builder_append(builder, "\"");
}

static char * amr_mqtt_bridge_builder_take(amr_mqtt_bridge_string_builder_t * builder)
{
  char * data = builder->data;
  builder->data = NULL;
  builder->length = 0U;
  builder->capacity = 0U;
  return data;
}

static double amr_mqtt_bridge_quaternion_to_yaw(
  double x,
  double y,
  double z,
  double w)
{
  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z));
  return atan2(siny_cosp, cosy_cosp);
}

static bool amr_mqtt_bridge_append_header(
  amr_mqtt_bridge_string_builder_t * builder,
  const std_msgs__msg__Header * header)
{
  return amr_mqtt_bridge_builder_appendf(
    builder,
    "\"header\":{\"stamp\":{\"sec\":%d,\"nanosec\":%u},\"frame_id\":",
    header->stamp.sec,
    header->stamp.nanosec) &&
    amr_mqtt_bridge_builder_append_json_string(builder, header->frame_id.data) &&
    amr_mqtt_bridge_builder_append(builder, "}");
}

static bool amr_mqtt_bridge_append_pose_fields(
  amr_mqtt_bridge_string_builder_t * builder,
  const geometry_msgs__msg__Pose * pose)
{
  const double yaw = amr_mqtt_bridge_quaternion_to_yaw(
    pose->orientation.x,
    pose->orientation.y,
    pose->orientation.z,
    pose->orientation.w);
  return amr_mqtt_bridge_builder_appendf(
    builder,
    "\"position\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
    "\"orientation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f,\"yaw\":%.6f}",
    pose->position.x,
    pose->position.y,
    pose->position.z,
    pose->orientation.x,
    pose->orientation.y,
    pose->orientation.z,
    pose->orientation.w,
    yaw);
}

static bool amr_mqtt_bridge_append_pose_stamped(
  amr_mqtt_bridge_string_builder_t * builder,
  const geometry_msgs__msg__PoseStamped * pose)
{
  if (!amr_mqtt_bridge_builder_append(builder, "{") ||
    !amr_mqtt_bridge_append_header(builder, &pose->header) ||
    !amr_mqtt_bridge_builder_append(builder, ",") ||
    !amr_mqtt_bridge_append_pose_fields(builder, &pose->pose) ||
    !amr_mqtt_bridge_builder_append(builder, "}"))
  {
    return false;
  }
  return true;
}

static char * amr_mqtt_bridge_serialize_pose_stamped(const void * message)
{
  const geometry_msgs__msg__PoseStamped * pose = (const geometry_msgs__msg__PoseStamped *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 256U) ||
    !amr_mqtt_bridge_append_pose_stamped(&builder, pose))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }
  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_path(const void * message)
{
  const nav_msgs__msg__Path * path = (const nav_msgs__msg__Path *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 1024U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &path->header) ||
    !amr_mqtt_bridge_builder_appendf(&builder, ",\"pose_count\":%zu,\"poses\":[", path->poses.size))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  for (size_t index = 0; index < path->poses.size; ++index) {
    if (index > 0U && !amr_mqtt_bridge_builder_append(&builder, ",")) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    if (!amr_mqtt_bridge_append_pose_stamped(&builder, &path->poses.data[index])) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
  }

  if (!amr_mqtt_bridge_builder_append(&builder, "]}")) {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_occupancy_grid(const void * message)
{
  const nav_msgs__msg__OccupancyGrid * grid = (const nav_msgs__msg__OccupancyGrid *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};
  const double origin_yaw = amr_mqtt_bridge_quaternion_to_yaw(
    grid->info.origin.orientation.x,
    grid->info.origin.orientation.y,
    grid->info.origin.orientation.z,
    grid->info.origin.orientation.w);

  if (!amr_mqtt_bridge_builder_init(&builder, 2048U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &grid->header) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"info\":{\"width\":%u,\"height\":%u,\"resolution\":%.6f,"
      "\"origin\":{\"position\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
      "\"orientation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f,\"yaw\":%.6f}}},\"data\":[",
      grid->info.width,
      grid->info.height,
      grid->info.resolution,
      grid->info.origin.position.x,
      grid->info.origin.position.y,
      grid->info.origin.position.z,
      grid->info.origin.orientation.x,
      grid->info.origin.orientation.y,
      grid->info.origin.orientation.z,
      grid->info.origin.orientation.w,
      origin_yaw))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  for (size_t index = 0; index < grid->data.size; ++index) {
    if (index > 0U && !amr_mqtt_bridge_builder_append(&builder, ",")) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    if (!amr_mqtt_bridge_builder_appendf(&builder, "%d", grid->data.data[index])) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
  }

  if (!amr_mqtt_bridge_builder_append(&builder, "]}")) {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_motion_status(const void * message)
{
  const amr_msgs__msg__MotionStatus * status = (const amr_msgs__msg__MotionStatus *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 512U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &status->header) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"command_id\":%u,\"active\":%s,\"goal_reached\":%s,\"obstacle_detected\":%s,"
      "\"remaining_distance\":%.6f,\"heading_error\":%.6f,\"current_pose\":",
      status->command_id,
      status->active ? "true" : "false",
      status->goal_reached ? "true" : "false",
      status->obstacle_detected ? "true" : "false",
      status->remaining_distance,
      status->heading_error) ||
    !amr_mqtt_bridge_append_pose_stamped(&builder, &status->current_pose) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_obstacle_report(const void * message)
{
  const amr_msgs__msg__ObstacleReport * report = (const amr_msgs__msg__ObstacleReport *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 512U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &report->header) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"active\":%s,\"is_dynamic\":%s,\"blocks_path\":%s,\"severity\":%u,"
      "\"distance\":%.6f,\"bearing\":%.6f,"
      "\"obstacle_point\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},\"source\":",
      report->active ? "true" : "false",
      report->is_dynamic ? "true" : "false",
      report->blocks_path ? "true" : "false",
      report->severity,
      report->distance,
      report->bearing,
      report->obstacle_point.x,
      report->obstacle_point.y,
      report->obstacle_point.z) ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, report->source.data) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static const char * amr_mqtt_bridge_skip_whitespace(const char * text)
{
  const char * cursor = text;
  while (cursor != NULL && (*cursor == ' ' || *cursor == '\n' || *cursor == '\r' || *cursor == '\t')) {
    ++cursor;
  }
  return cursor;
}

static const char * amr_mqtt_bridge_find_matching_delimiter(
  const char * start,
  char open_char,
  char close_char)
{
  int depth = 0;
  const char * cursor = start;

  if (start == NULL || *start != open_char) {
    return NULL;
  }

  while (*cursor != '\0') {
    if (*cursor == '"') {
      ++cursor;
      while (*cursor != '\0') {
        if (*cursor == '\\' && cursor[1] != '\0') {
          cursor += 2;
          continue;
        }
        if (*cursor == '"') {
          break;
        }
        ++cursor;
      }
    } else if (*cursor == open_char) {
      ++depth;
    } else if (*cursor == close_char) {
      --depth;
      if (depth == 0) {
        return cursor;
      }
    }
    ++cursor;
  }

  return NULL;
}

static const char * amr_mqtt_bridge_find_key_in_range(
  const char * begin,
  const char * end,
  const char * key)
{
  char pattern[128] = {0};
  const char * cursor = begin;
  int pattern_length = snprintf(pattern, sizeof(pattern), "\"%s\"", key);

  if (begin == NULL || end == NULL || key == NULL || pattern_length <= 0) {
    return NULL;
  }

  while (cursor < end) {
    const char * match = strstr(cursor, pattern);
    if (match == NULL || match >= end) {
      return NULL;
    }
    return match;
  }

  return NULL;
}

static const char * amr_mqtt_bridge_find_value_for_key(
  const char * begin,
  const char * end,
  const char * key)
{
  const char * key_pos = amr_mqtt_bridge_find_key_in_range(begin, end, key);
  const char * cursor = NULL;

  if (key_pos == NULL) {
    return NULL;
  }

  cursor = key_pos;
  while (cursor < end && *cursor != ':') {
    ++cursor;
  }
  if (cursor >= end || *cursor != ':') {
    return NULL;
  }

  ++cursor;
  cursor = amr_mqtt_bridge_skip_whitespace(cursor);
  if (cursor >= end) {
    return NULL;
  }

  return cursor;
}

static bool amr_mqtt_bridge_extract_json_string_in_range(
  const char * begin,
  const char * end,
  const char * key,
  char * output,
  size_t output_capacity)
{
  const char * value = amr_mqtt_bridge_find_value_for_key(begin, end, key);
  const char * cursor = NULL;
  size_t length = 0U;

  if (value == NULL || *value != '"' || output == NULL || output_capacity == 0U) {
    return false;
  }

  cursor = value + 1;
  while (cursor < end && *cursor != '"') {
    if (*cursor == '\\' && (cursor + 1) < end) {
      cursor += 2;
      continue;
    }
    ++cursor;
  }

  if (cursor >= end || *cursor != '"') {
    return false;
  }

  length = (size_t)(cursor - (value + 1));
  if (length >= output_capacity) {
    length = output_capacity - 1U;
  }

  memcpy(output, value + 1, length);
  output[length] = '\0';
  return true;
}

static bool amr_mqtt_bridge_extract_json_double_in_range(
  const char * begin,
  const char * end,
  const char * key,
  double * output)
{
  const char * value = amr_mqtt_bridge_find_value_for_key(begin, end, key);
  char * parse_end = NULL;

  if (value == NULL || output == NULL) {
    return false;
  }

  *output = strtod(value, &parse_end);
  return parse_end != value;
}

static bool amr_mqtt_bridge_extract_json_object_in_range(
  const char * begin,
  const char * end,
  const char * key,
  const char ** object_begin,
  const char ** object_end)
{
  const char * value = amr_mqtt_bridge_find_value_for_key(begin, end, key);
  const char * close = NULL;

  if (value == NULL || *value != '{' || object_begin == NULL || object_end == NULL) {
    return false;
  }

  close = amr_mqtt_bridge_find_matching_delimiter(value, '{', '}');
  if (close == NULL || close > end) {
    return false;
  }

  *object_begin = value;
  *object_end = close + 1;
  return true;
}

static bool amr_mqtt_bridge_extract_json_array_in_range(
  const char * begin,
  const char * end,
  const char * key,
  const char ** array_begin,
  const char ** array_end)
{
  const char * value = amr_mqtt_bridge_find_value_for_key(begin, end, key);
  const char * close = NULL;

  if (value == NULL || *value != '[' || array_begin == NULL || array_end == NULL) {
    return false;
  }

  close = amr_mqtt_bridge_find_matching_delimiter(value, '[', ']');
  if (close == NULL || close > end) {
    return false;
  }

  *array_begin = value;
  *array_end = close + 1;
  return true;
}

static bool amr_mqtt_bridge_parse_pose_stamped_in_range(
  const char * begin,
  const char * end,
  geometry_msgs__msg__PoseStamped * pose)
{
  const char * header_begin = NULL;
  const char * header_end = NULL;
  const char * position_begin = NULL;
  const char * position_end = NULL;
  const char * orientation_begin = NULL;
  const char * orientation_end = NULL;
  char frame_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = "map";

  if (pose == NULL) {
    return false;
  }

  if (amr_mqtt_bridge_extract_json_object_in_range(begin, end, "header", &header_begin, &header_end)) {
    (void)amr_mqtt_bridge_extract_json_string_in_range(
      header_begin, header_end, "frame_id", frame_id, sizeof(frame_id));
  }

  if (!amr_mqtt_bridge_extract_json_object_in_range(begin, end, "position", &position_begin, &position_end)) {
    const char * pose_begin = NULL;
    const char * pose_end = NULL;
    if (!amr_mqtt_bridge_extract_json_object_in_range(begin, end, "pose", &pose_begin, &pose_end) ||
      !amr_mqtt_bridge_extract_json_object_in_range(pose_begin, pose_end, "position", &position_begin, &position_end))
    {
      return false;
    }
    if (!amr_mqtt_bridge_extract_json_object_in_range(pose_begin, pose_end, "orientation", &orientation_begin, &orientation_end)) {
      return false;
    }
  } else if (!amr_mqtt_bridge_extract_json_object_in_range(begin, end, "orientation", &orientation_begin, &orientation_end)) {
    return false;
  }

  if (!amr_mqtt_bridge_extract_json_double_in_range(position_begin, position_end, "x", &pose->pose.position.x) ||
    !amr_mqtt_bridge_extract_json_double_in_range(position_begin, position_end, "y", &pose->pose.position.y) ||
    !amr_mqtt_bridge_extract_json_double_in_range(position_begin, position_end, "z", &pose->pose.position.z) ||
    !amr_mqtt_bridge_extract_json_double_in_range(orientation_begin, orientation_end, "x", &pose->pose.orientation.x) ||
    !amr_mqtt_bridge_extract_json_double_in_range(orientation_begin, orientation_end, "y", &pose->pose.orientation.y) ||
    !amr_mqtt_bridge_extract_json_double_in_range(orientation_begin, orientation_end, "z", &pose->pose.orientation.z) ||
    !amr_mqtt_bridge_extract_json_double_in_range(orientation_begin, orientation_end, "w", &pose->pose.orientation.w))
  {
    return false;
  }

  pose->header.stamp.sec = 0;
  pose->header.stamp.nanosec = 0U;
  rosidl_runtime_c__String__assign(&pose->header.frame_id, frame_id);
  return true;
}

static size_t amr_mqtt_bridge_count_objects_in_array(const char * begin, const char * end)
{
  const char * cursor = begin;
  size_t count = 0U;
  int array_depth = 0;

  while (cursor < end) {
    if (*cursor == '[') {
      ++array_depth;
    } else if (*cursor == ']') {
      --array_depth;
    } else if (*cursor == '{' && array_depth == 1) {
      const char * object_end = amr_mqtt_bridge_find_matching_delimiter(cursor, '{', '}');
      if (object_end == NULL || object_end > end) {
        break;
      }
      ++count;
      cursor = object_end + 1;
      continue;
    }
    ++cursor;
  }

  return count;
}

static bool amr_mqtt_bridge_parse_waypoints_array(
  const char * begin,
  const char * end,
  geometry_msgs__msg__PoseStamped__Sequence * waypoints)
{
  const char * cursor = begin;
  size_t index = 0U;
  size_t count = amr_mqtt_bridge_count_objects_in_array(begin, end);
  int array_depth = 0;

  if (!geometry_msgs__msg__PoseStamped__Sequence__init(waypoints, count)) {
    return false;
  }

  while (cursor < end && index < count) {
    if (*cursor == '[') {
      ++array_depth;
    } else if (*cursor == ']') {
      --array_depth;
    } else if (*cursor == '{' && array_depth == 1) {
      const char * object_end = amr_mqtt_bridge_find_matching_delimiter(cursor, '{', '}');
      if (object_end == NULL || object_end > end) {
        geometry_msgs__msg__PoseStamped__Sequence__fini(waypoints);
        return false;
      }
      if (!amr_mqtt_bridge_parse_pose_stamped_in_range(cursor, object_end + 1, &waypoints->data[index])) {
        geometry_msgs__msg__PoseStamped__Sequence__fini(waypoints);
        return false;
      }
      ++index;
      cursor = object_end + 1;
      continue;
    }
    ++cursor;
  }

  return true;
}

static void amr_mqtt_bridge_fill_quaternion_from_yaw(
  double yaw,
  geometry_msgs__msg__Quaternion * quaternion)
{
  quaternion->x = 0.0;
  quaternion->y = 0.0;
  quaternion->z = sin(yaw * 0.5);
  quaternion->w = cos(yaw * 0.5);
}

static void amr_mqtt_bridge_format_uuid(
  const uint8_t uuid[16],
  char * output,
  size_t output_capacity)
{
  (void)snprintf(
    output,
    output_capacity,
    "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
    uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7],
    uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
}

static void amr_mqtt_bridge_generate_goal_uuid(uint8_t uuid[16])
{
  static uint32_t sequence = 0U;
  struct timespec now = {0};
  uint64_t stamp = 0U;

  (void)clock_gettime(CLOCK_REALTIME, &now);
  stamp = ((uint64_t)now.tv_sec << 32) ^ (uint64_t)now.tv_nsec;
  ++sequence;

  for (size_t index = 0; index < 8U; ++index) {
    uuid[index] = (uint8_t)((stamp >> (index * 8U)) & 0xffU);
  }
  for (size_t index = 0; index < 4U; ++index) {
    uuid[8U + index] = (uint8_t)((sequence >> (index * 8U)) & 0xffU);
  }
  uuid[12] = 0x41U;
  uuid[13] = 0x4dU;
  uuid[14] = 0x52U;
  uuid[15] = 0x30U;
}

static bool amr_mqtt_bridge_uuid_equals(const uint8_t lhs[16], const uint8_t rhs[16])
{
  return memcmp(lhs, rhs, 16U) == 0;
}

static char * amr_mqtt_bridge_serialize_simple_response(
  const char * request_id,
  bool success,
  const char * message)
{
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 256U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"request_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, request_id) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"success\":%s,\"message\":",
      success ? "true" : "false") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, message) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_plan_segment_response(
  const char * request_id,
  const amr_msgs__srv__PlanSegment_Response * response)
{
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 1024U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"request_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, request_id) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"success\":%s,\"message\":",
      response->success ? "true" : "false") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, response->message.data) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"plan\":"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  {
    char * plan_payload = amr_mqtt_bridge_serialize_path(&response->plan);
    if (plan_payload == NULL) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    if (!amr_mqtt_bridge_builder_append(&builder, plan_payload) ||
      !amr_mqtt_bridge_builder_append(&builder, "}"))
    {
      free(plan_payload);
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    free(plan_payload);
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_plan_route_response(
  const char * request_id,
  const amr_msgs__srv__PlanRoute_Response * response)
{
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 1024U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"request_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, request_id) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"success\":%s,\"message\":",
      response->success ? "true" : "false") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, response->message.data) ||
    !amr_mqtt_bridge_builder_appendf(&builder, ",\"plans_count\":%zu,\"plans\":[", response->plans.size))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  for (size_t index = 0; index < response->plans.size; ++index) {
    char * plan_payload = NULL;
    if (index > 0U && !amr_mqtt_bridge_builder_append(&builder, ",")) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    plan_payload = amr_mqtt_bridge_serialize_path(&response->plans.data[index]);
    if (plan_payload == NULL) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    if (!amr_mqtt_bridge_builder_append(&builder, plan_payload)) {
      free(plan_payload);
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    free(plan_payload);
  }

  if (!amr_mqtt_bridge_builder_append(&builder, "]}")) {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_navigate_goal_response(
  const char * request_id,
  const uint8_t goal_uuid[16],
  const amr_msgs__action__NavigateToPose_SendGoal_Response * response,
  const char * message)
{
  char uuid_text[64] = {0};
  amr_mqtt_bridge_string_builder_t builder = {0};
  amr_mqtt_bridge_format_uuid(goal_uuid, uuid_text, sizeof(uuid_text));

  if (!amr_mqtt_bridge_builder_init(&builder, 256U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"request_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, request_id) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"accepted\":%s,\"goal_id\":",
      response->accepted ? "true" : "false") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, uuid_text) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"stamp\":{\"sec\":%d,\"nanosec\":%u},\"message\":",
      response->stamp.sec,
      response->stamp.nanosec) ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, message) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_navigate_result_response(
  const char * request_id,
  const uint8_t goal_uuid[16],
  const amr_msgs__action__NavigateToPose_GetResult_Response * response)
{
  char uuid_text[64] = {0};
  amr_mqtt_bridge_string_builder_t builder = {0};
  amr_mqtt_bridge_format_uuid(goal_uuid, uuid_text, sizeof(uuid_text));

  if (!amr_mqtt_bridge_builder_init(&builder, 256U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"request_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, request_id) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"goal_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, uuid_text) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"status\":%d,\"success\":%s,\"message\":",
      response->status,
      response->result.success ? "true" : "false") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, response->result.message.data) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_navigate_feedback_message(
  const char * request_id,
  const amr_msgs__action__NavigateToPose_FeedbackMessage * feedback_message)
{
  char uuid_text[64] = {0};
  amr_mqtt_bridge_string_builder_t builder = {0};
  amr_mqtt_bridge_format_uuid(feedback_message->goal_id.uuid, uuid_text, sizeof(uuid_text));

  if (!amr_mqtt_bridge_builder_init(&builder, 512U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"request_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, request_id) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"goal_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, uuid_text) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"current_pose\":"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  {
    char * pose_payload = amr_mqtt_bridge_serialize_pose_stamped(&feedback_message->feedback.current_pose);
    if (pose_payload == NULL) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    if (!amr_mqtt_bridge_builder_append(&builder, pose_payload) ||
      !amr_mqtt_bridge_builder_appendf(
        &builder,
        ",\"remaining_distance\":%.6f,\"heading_error\":%.6f}",
        feedback_message->feedback.remaining_distance,
        feedback_message->feedback.heading_error))
    {
      free(pose_payload);
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    free(pose_payload);
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static bool amr_mqtt_bridge_wait_for_service_response(
  rcl_client_t * client,
  void * response,
  int64_t expected_sequence_number,
  int timeout_ms)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  int elapsed_ms = 0;
  rcl_ret_t rc = rcl_wait_set_init(
    &wait_set,
    0U,
    0U,
    0U,
    1U,
    0U,
    0U,
    &g_amr_mqtt_bridge_runtime.support.context,
    g_amr_mqtt_bridge_runtime.allocator);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return false;
  }

  while (elapsed_ms < timeout_ms) {
    rmw_request_id_t response_header;
    memset(&response_header, 0, sizeof(response_header));
    rc = rcl_wait_set_clear(&wait_set);
    if (rc != RCL_RET_OK) {
      break;
    }
    rc = rcl_wait_set_add_client(&wait_set, client, NULL);
    if (rc != RCL_RET_OK) {
      break;
    }
    rc = rcl_wait(&wait_set, 100 * 1000 * 1000);
    if (rc == RCL_RET_TIMEOUT) {
      elapsed_ms += 100;
      continue;
    }
    if (rc != RCL_RET_OK) {
      break;
    }
    rc = rcl_take_response(client, &response_header, response);
    if (rc == RCL_RET_OK && response_header.sequence_number == expected_sequence_number) {
      amr_mqtt_bridge_log_rcl_fini_error("service wait set", rcl_wait_set_fini(&wait_set));
      return true;
    }
    if (rc != RCL_RET_OK && rc != RCL_RET_CLIENT_TAKE_FAILED) {
      break;
    }
    if (rc != RCL_RET_OK) {
      rcl_reset_error();
    }
    elapsed_ms += 100;
  }

  amr_mqtt_bridge_log_rcl_fini_error("service wait set", rcl_wait_set_fini(&wait_set));
  return false;
}

static bool amr_mqtt_bridge_publish_payload(
  const char * mqtt_topic,
  const char * payload,
  int qos)
{
  size_t payload_length = strlen(payload);

  if (!amr_mqtt_bridge_ensure_connected()) {
    RCUTILS_LOG_WARN_NAMED(
      "amr_mqtt_bridge",
      "Skipping publish for '%s' because MQTT is disconnected",
      mqtt_topic);
    return false;
  }

  MQTTClient_message message = MQTTClient_message_initializer;
  MQTTClient_deliveryToken token = 0;
  message.payload = (void *)payload;
  message.payloadlen = (int)payload_length;
  message.qos = qos;
  message.retained = 0;

  int mqtt_rc = MQTTClient_publishMessage(
    g_amr_mqtt_bridge_mqtt.client,
    mqtt_topic,
    &message,
    &token);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    if (mqtt_rc == MQTTCLIENT_DISCONNECTED) {
      g_amr_mqtt_bridge_mqtt.connected = false;
    }
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Failed to publish MQTT message on '%s': rc=%d (%s), payload_bytes=%zu",
      mqtt_topic,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc),
      payload_length);
    return false;
  }

  if (qos > 0) {
    (void)MQTTClient_waitForCompletion(g_amr_mqtt_bridge_mqtt.client, token, 1000L);
  }

  return true;
}

static void amr_mqtt_bridge_publish_simple_response(
  const char * mqtt_topic,
  const char * request_id,
  bool success,
  const char * message)
{
  char * payload = amr_mqtt_bridge_serialize_simple_response(request_id, success, message);
  if (payload == NULL) {
    return;
  }
  (void)amr_mqtt_bridge_publish_payload(
    mqtt_topic,
    payload,
    g_amr_mqtt_bridge_config.mqtt.service_qos);
  free(payload);
}

static bool amr_mqtt_bridge_wait_for_service_available(rcl_client_t * client, int timeout_ms)
{
  int elapsed_ms = 0;
  while (elapsed_ms < timeout_ms) {
    bool is_available = false;
    rcl_ret_t rc = rcl_service_server_is_available(
      &g_amr_mqtt_bridge_runtime.node,
      client,
      &is_available);
    if (rc == RCL_RET_OK && is_available) {
      return true;
    }
    if (rc != RCL_RET_OK) {
      rcl_reset_error();
      return false;
    }
    elapsed_ms += 100;
    struct timespec sleep_time = {.tv_sec = 0, .tv_nsec = 100 * 1000 * 1000};
    nanosleep(&sleep_time, NULL);
  }
  return false;
}

static void amr_mqtt_bridge_handle_set_initial_pose_command(const char * payload)
{
  geometry_msgs__msg__PoseWithCovarianceStamped initial_pose;
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  char frame_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = "map";
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double covariance_x = 0.25;
  double covariance_y = 0.25;
  double covariance_yaw = 0.06853891945200942;
  rcl_ret_t rc;

  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id)) ||
    !amr_mqtt_bridge_extract_json_double_in_range(payload, payload + strlen(payload), "x", &x) ||
    !amr_mqtt_bridge_extract_json_double_in_range(payload, payload + strlen(payload), "y", &y) ||
    !amr_mqtt_bridge_extract_json_double_in_range(payload, payload + strlen(payload), "yaw", &yaw))
  {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose,
      request_id[0] != '\0' ? request_id : "",
      false,
      "invalid set_initial_pose payload");
    return;
  }

  (void)amr_mqtt_bridge_extract_json_string_in_range(
    payload, payload + strlen(payload), "frame_id", frame_id, sizeof(frame_id));
  (void)amr_mqtt_bridge_extract_json_double_in_range(payload, payload + strlen(payload), "covariance_x", &covariance_x);
  (void)amr_mqtt_bridge_extract_json_double_in_range(payload, payload + strlen(payload), "covariance_y", &covariance_y);
  (void)amr_mqtt_bridge_extract_json_double_in_range(payload, payload + strlen(payload), "covariance_yaw", &covariance_yaw);

  memset(&initial_pose, 0, sizeof(initial_pose));
  if (!geometry_msgs__msg__PoseWithCovarianceStamped__init(&initial_pose)) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose,
      request_id,
      false,
      "failed to allocate initial pose message");
    return;
  }

  initial_pose.header.stamp.sec = 0;
  initial_pose.header.stamp.nanosec = 0U;
  (void)rosidl_runtime_c__String__assign(&initial_pose.header.frame_id, frame_id);
  initial_pose.pose.pose.position.x = x;
  initial_pose.pose.pose.position.y = y;
  initial_pose.pose.pose.position.z = 0.0;
  amr_mqtt_bridge_fill_quaternion_from_yaw(yaw, &initial_pose.pose.pose.orientation);
  for (size_t index = 0; index < 36U; ++index) {
    initial_pose.pose.covariance[index] = 0.0;
  }
  initial_pose.pose.covariance[0] = covariance_x;
  initial_pose.pose.covariance[7] = covariance_y;
  initial_pose.pose.covariance[35] = covariance_yaw;

  rc = rcl_publish(&g_amr_mqtt_bridge_ros_state.initial_pose_publisher, &initial_pose, NULL);
  geometry_msgs__msg__PoseWithCovarianceStamped__fini(&initial_pose);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose,
      request_id,
      false,
      "failed to publish initial pose");
    return;
  }

  amr_mqtt_bridge_publish_simple_response(
    g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose,
    request_id,
    true,
    "initial pose published");
}

static void amr_mqtt_bridge_handle_plan_segment_request(const char * payload)
{
  amr_msgs__srv__PlanSegment_Request request;
  amr_msgs__srv__PlanSegment_Response response;
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  const char * start_begin = NULL;
  const char * start_end = NULL;
  const char * goal_begin = NULL;
  const char * goal_end = NULL;
  int64_t sequence_number = 0;
  rcl_ret_t rc;
  char * response_payload = NULL;

  memset(&request, 0, sizeof(request));
  memset(&response, 0, sizeof(response));
  if (!amr_msgs__srv__PlanSegment_Request__init(&request) ||
    !amr_msgs__srv__PlanSegment_Response__init(&response))
  {
    goto fail;
  }

  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id)) ||
    !amr_mqtt_bridge_extract_json_object_in_range(payload, payload + strlen(payload), "start", &start_begin, &start_end) ||
    !amr_mqtt_bridge_extract_json_object_in_range(payload, payload + strlen(payload), "goal", &goal_begin, &goal_end) ||
    !amr_mqtt_bridge_parse_pose_stamped_in_range(start_begin, start_end, &request.start) ||
    !amr_mqtt_bridge_parse_pose_stamped_in_range(goal_begin, goal_end, &request.goal))
  {
    goto fail;
  }

  if (!amr_mqtt_bridge_wait_for_service_available(&g_amr_mqtt_bridge_ros_state.plan_segment_client, 1000)) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
      request_id,
      false,
      "plan_segment service unavailable");
    goto done;
  }

  rc = rcl_send_request(&g_amr_mqtt_bridge_ros_state.plan_segment_client, &request, &sequence_number);
  if (rc != RCL_RET_OK ||
    !amr_mqtt_bridge_wait_for_service_response(
      &g_amr_mqtt_bridge_ros_state.plan_segment_client,
      &response,
      sequence_number,
      3000))
  {
    rcl_reset_error();
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
      request_id,
      false,
      "plan_segment request failed or timed out");
    goto done;
  }

  response_payload = amr_mqtt_bridge_serialize_plan_segment_response(request_id, &response);
  if (response_payload != NULL) {
    (void)amr_mqtt_bridge_publish_payload(
      g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
      response_payload,
      g_amr_mqtt_bridge_config.mqtt.service_qos);
    free(response_payload);
  }

done:
  amr_msgs__srv__PlanSegment_Request__fini(&request);
  amr_msgs__srv__PlanSegment_Response__fini(&response);
  return;

fail:
  amr_mqtt_bridge_publish_simple_response(
    g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
    request_id[0] != '\0' ? request_id : "",
    false,
    "invalid plan_segment payload");
  amr_msgs__srv__PlanSegment_Request__fini(&request);
  amr_msgs__srv__PlanSegment_Response__fini(&response);
}

static void amr_mqtt_bridge_handle_plan_route_request(const char * payload)
{
  amr_msgs__srv__PlanRoute_Request request;
  amr_msgs__srv__PlanRoute_Response response;
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  const char * start_begin = NULL;
  const char * start_end = NULL;
  const char * waypoints_begin = NULL;
  const char * waypoints_end = NULL;
  int64_t sequence_number = 0;
  rcl_ret_t rc;
  char * response_payload = NULL;

  memset(&request, 0, sizeof(request));
  memset(&response, 0, sizeof(response));
  if (!amr_msgs__srv__PlanRoute_Request__init(&request) ||
    !amr_msgs__srv__PlanRoute_Response__init(&response))
  {
    goto fail;
  }

  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id)) ||
    !amr_mqtt_bridge_extract_json_object_in_range(payload, payload + strlen(payload), "start", &start_begin, &start_end) ||
    !amr_mqtt_bridge_extract_json_array_in_range(payload, payload + strlen(payload), "waypoints", &waypoints_begin, &waypoints_end) ||
    !amr_mqtt_bridge_parse_pose_stamped_in_range(start_begin, start_end, &request.start) ||
    !amr_mqtt_bridge_parse_waypoints_array(waypoints_begin, waypoints_end, &request.waypoints))
  {
    goto fail;
  }

  if (!amr_mqtt_bridge_wait_for_service_available(&g_amr_mqtt_bridge_ros_state.plan_route_client, 1000)) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_route,
      request_id,
      false,
      "plan_route service unavailable");
    goto done;
  }

  rc = rcl_send_request(&g_amr_mqtt_bridge_ros_state.plan_route_client, &request, &sequence_number);
  if (rc != RCL_RET_OK ||
    !amr_mqtt_bridge_wait_for_service_response(
      &g_amr_mqtt_bridge_ros_state.plan_route_client,
      &response,
      sequence_number,
      3000))
  {
    rcl_reset_error();
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_route,
      request_id,
      false,
      "plan_route request failed or timed out");
    goto done;
  }

  response_payload = amr_mqtt_bridge_serialize_plan_route_response(request_id, &response);
  if (response_payload != NULL) {
    (void)amr_mqtt_bridge_publish_payload(
      g_amr_mqtt_bridge_config.mqtt.response_plan_route,
      response_payload,
      g_amr_mqtt_bridge_config.mqtt.service_qos);
    free(response_payload);
  }

done:
  amr_msgs__srv__PlanRoute_Request__fini(&request);
  amr_msgs__srv__PlanRoute_Response__fini(&response);
  return;

fail:
  amr_mqtt_bridge_publish_simple_response(
    g_amr_mqtt_bridge_config.mqtt.response_plan_route,
    request_id[0] != '\0' ? request_id : "",
    false,
    "invalid plan_route payload");
  amr_msgs__srv__PlanRoute_Request__fini(&request);
  amr_msgs__srv__PlanRoute_Response__fini(&response);
}

static void amr_mqtt_bridge_handle_navigate_to_pose_command(const char * payload)
{
  amr_msgs__action__NavigateToPose_SendGoal_Request request;
  amr_msgs__action__NavigateToPose_SendGoal_Response response;
  const char * goal_begin = NULL;
  const char * goal_end = NULL;
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  bool is_available = false;
  int64_t sequence_number = 0;
  rcl_ret_t rc;
  char * response_payload = NULL;

  memset(&request, 0, sizeof(request));
  memset(&response, 0, sizeof(response));
  if (!amr_msgs__action__NavigateToPose_SendGoal_Request__init(&request)) {
    return;
  }

  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id)) ||
    !amr_mqtt_bridge_extract_json_object_in_range(payload, payload + strlen(payload), "goal_pose", &goal_begin, &goal_end) ||
    !amr_mqtt_bridge_parse_pose_stamped_in_range(goal_begin, goal_end, &request.goal.goal_pose))
  {
    response.accepted = false;
    response_payload = amr_mqtt_bridge_serialize_navigate_goal_response("", g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid, &response, "invalid navigate_to_pose payload");
    if (response_payload != NULL) {
      (void)amr_mqtt_bridge_publish_payload(
        g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
        response_payload,
        g_amr_mqtt_bridge_config.mqtt.command_qos);
      free(response_payload);
    }
    amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
    return;
  }

  if (g_amr_mqtt_bridge_ros_state.navigate_state.active) {
    response.accepted = false;
    response_payload = amr_mqtt_bridge_serialize_navigate_goal_response(
      request_id,
      g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid,
      &response,
      "navigate_to_pose goal already active");
    if (response_payload != NULL) {
      (void)amr_mqtt_bridge_publish_payload(
        g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
        response_payload,
        g_amr_mqtt_bridge_config.mqtt.command_qos);
      free(response_payload);
    }
    amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
    return;
  }

  rc = rcl_action_server_is_available(
    &g_amr_mqtt_bridge_runtime.node,
    &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
    &is_available);
  if (rc != RCL_RET_OK || !is_available) {
    rcl_reset_error();
    response.accepted = false;
    response_payload = amr_mqtt_bridge_serialize_navigate_goal_response(
      request_id,
      g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid,
      &response,
      "navigate_to_pose action unavailable");
    if (response_payload != NULL) {
      (void)amr_mqtt_bridge_publish_payload(
        g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
        response_payload,
        g_amr_mqtt_bridge_config.mqtt.command_qos);
      free(response_payload);
    }
    amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
    return;
  }

  amr_mqtt_bridge_generate_goal_uuid(g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid);
  memcpy(request.goal_id.uuid, g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid, 16U);
  rc = rcl_action_send_goal_request(
    &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
    &request,
    &sequence_number);
  amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    response.accepted = false;
    response_payload = amr_mqtt_bridge_serialize_navigate_goal_response(
      request_id,
      g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid,
      &response,
      "failed to send navigate_to_pose goal");
    if (response_payload != NULL) {
      (void)amr_mqtt_bridge_publish_payload(
        g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
        response_payload,
        g_amr_mqtt_bridge_config.mqtt.command_qos);
      free(response_payload);
    }
    return;
  }

  g_amr_mqtt_bridge_ros_state.navigate_state.active = true;
  g_amr_mqtt_bridge_ros_state.navigate_state.goal_response_pending = true;
  g_amr_mqtt_bridge_ros_state.navigate_state.result_response_pending = false;
  g_amr_mqtt_bridge_ros_state.navigate_state.goal_request_sequence_number = sequence_number;
  g_amr_mqtt_bridge_ros_state.navigate_state.result_request_sequence_number = 0;
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_ros_state.navigate_state.request_id,
    sizeof(g_amr_mqtt_bridge_ros_state.navigate_state.request_id),
    request_id);
}

static void amr_mqtt_bridge_handle_mqtt_message(const char * topic, const char * payload)
{
  if (strcmp(topic, g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose) == 0) {
    amr_mqtt_bridge_handle_set_initial_pose_command(payload);
  } else if (strcmp(topic, g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose) == 0) {
    amr_mqtt_bridge_handle_navigate_to_pose_command(payload);
  } else if (strcmp(topic, g_amr_mqtt_bridge_config.mqtt.request_plan_segment) == 0) {
    amr_mqtt_bridge_handle_plan_segment_request(payload);
  } else if (strcmp(topic, g_amr_mqtt_bridge_config.mqtt.request_plan_route) == 0) {
    amr_mqtt_bridge_handle_plan_route_request(payload);
  }
}

static void amr_mqtt_bridge_poll_mqtt(void)
{
  int mqtt_rc = 0;
  int processed_count = 0;

  if (!amr_mqtt_bridge_ensure_connected()) {
    return;
  }

  do {
    char * topic_name = NULL;
    int topic_length = 0;
    MQTTClient_message * message = NULL;
    mqtt_rc = MQTTClient_receive(
      g_amr_mqtt_bridge_mqtt.client,
      &topic_name,
      &topic_length,
      &message,
      0UL);
    if (mqtt_rc != MQTTCLIENT_SUCCESS) {
      if (mqtt_rc == MQTTCLIENT_DISCONNECTED) {
        g_amr_mqtt_bridge_mqtt.connected = false;
      }
      break;
    }
    if (message == NULL || topic_name == NULL) {
      break;
    }

    {
      char * payload_text = (char *)calloc((size_t)message->payloadlen + 1U, sizeof(char));
      if (payload_text != NULL) {
        memcpy(payload_text, message->payload, (size_t)message->payloadlen);
        payload_text[message->payloadlen] = '\0';
        amr_mqtt_bridge_handle_mqtt_message(topic_name, payload_text);
        free(payload_text);
      }
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topic_name);
    ++processed_count;
  } while (processed_count < 8);
}

static int amr_mqtt_bridge_init_command_interfaces(void)
{
  rcl_ret_t rc;
  size_t num_subscriptions = 0U;
  size_t num_guard_conditions = 0U;
  size_t num_timers = 0U;
  size_t num_clients = 0U;
  size_t num_services = 0U;

  g_amr_mqtt_bridge_ros_state.initial_pose_publisher = rcl_get_zero_initialized_publisher();
  g_amr_mqtt_bridge_ros_state.plan_segment_client = rcl_get_zero_initialized_client();
  g_amr_mqtt_bridge_ros_state.plan_route_client = rcl_get_zero_initialized_client();
  g_amr_mqtt_bridge_ros_state.navigate_to_pose_client = rcl_action_get_zero_initialized_client();
  g_amr_mqtt_bridge_ros_state.navigate_wait_set = rcl_get_zero_initialized_wait_set();

  rc = rclc_publisher_init_default(
    &g_amr_mqtt_bridge_ros_state.initial_pose_publisher,
    &g_amr_mqtt_bridge_runtime.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped),
    g_amr_mqtt_bridge_config.ros.topic_initial_pose);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }
  g_amr_mqtt_bridge_ros_state.initial_pose_publisher_initialized = true;

  {
    rcl_client_options_t client_options = rcl_client_get_default_options();
    rc = rcl_client_init(
      &g_amr_mqtt_bridge_ros_state.plan_segment_client,
      &g_amr_mqtt_bridge_runtime.node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(amr_msgs, srv, PlanSegment),
      g_amr_mqtt_bridge_config.ros.service_plan_segment,
      &client_options);
  }
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }
  g_amr_mqtt_bridge_ros_state.plan_segment_client_initialized = true;

  {
    rcl_client_options_t client_options = rcl_client_get_default_options();
    rc = rcl_client_init(
      &g_amr_mqtt_bridge_ros_state.plan_route_client,
      &g_amr_mqtt_bridge_runtime.node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(amr_msgs, srv, PlanRoute),
      g_amr_mqtt_bridge_config.ros.service_plan_route,
      &client_options);
  }
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }
  g_amr_mqtt_bridge_ros_state.plan_route_client_initialized = true;

  {
    rcl_action_client_options_t action_options = rcl_action_client_get_default_options();
    rc = rcl_action_client_init(
      &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
      &g_amr_mqtt_bridge_runtime.node,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(amr_msgs, NavigateToPose),
      g_amr_mqtt_bridge_config.ros.action_navigate_to_pose,
      &action_options);
  }
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }
  g_amr_mqtt_bridge_ros_state.navigate_to_pose_client_initialized = true;

  rc = rcl_action_client_wait_set_get_num_entities(
    &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
    &num_subscriptions,
    &num_guard_conditions,
    &num_timers,
    &num_clients,
    &num_services);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }

  rc = rcl_wait_set_init(
    &g_amr_mqtt_bridge_ros_state.navigate_wait_set,
    num_subscriptions,
    num_guard_conditions,
    num_timers,
    num_clients,
    num_services,
    0U,
    &g_amr_mqtt_bridge_runtime.support.context,
    g_amr_mqtt_bridge_runtime.allocator);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }
  g_amr_mqtt_bridge_ros_state.navigate_wait_set_initialized = true;

  if (!action_msgs__msg__GoalStatusArray__init(&g_amr_mqtt_bridge_ros_state.navigate_status_message)) {
    return 1;
  }
  g_amr_mqtt_bridge_ros_state.navigate_status_message_initialized = true;

  memset(&g_amr_mqtt_bridge_ros_state.navigate_state, 0, sizeof(g_amr_mqtt_bridge_ros_state.navigate_state));
  return 0;
}

static void amr_mqtt_bridge_fini_command_interfaces(void)
{
  if (g_amr_mqtt_bridge_ros_state.navigate_wait_set_initialized) {
    amr_mqtt_bridge_log_rcl_fini_error(
      "navigate wait set",
      rcl_wait_set_fini(&g_amr_mqtt_bridge_ros_state.navigate_wait_set));
    g_amr_mqtt_bridge_ros_state.navigate_wait_set_initialized = false;
  }

  if (g_amr_mqtt_bridge_ros_state.navigate_to_pose_client_initialized) {
    amr_mqtt_bridge_log_rcl_fini_error(
      "navigate action client",
      rcl_action_client_fini(
        &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
        &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.navigate_to_pose_client_initialized = false;
  }

  if (g_amr_mqtt_bridge_ros_state.plan_route_client_initialized) {
    amr_mqtt_bridge_log_rcl_fini_error(
      "plan_route client",
      rcl_client_fini(&g_amr_mqtt_bridge_ros_state.plan_route_client, &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.plan_route_client_initialized = false;
  }

  if (g_amr_mqtt_bridge_ros_state.plan_segment_client_initialized) {
    amr_mqtt_bridge_log_rcl_fini_error(
      "plan_segment client",
      rcl_client_fini(&g_amr_mqtt_bridge_ros_state.plan_segment_client, &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.plan_segment_client_initialized = false;
  }

  if (g_amr_mqtt_bridge_ros_state.initial_pose_publisher_initialized) {
    amr_mqtt_bridge_log_rcl_fini_error(
      "initial_pose publisher",
      rcl_publisher_fini(&g_amr_mqtt_bridge_ros_state.initial_pose_publisher, &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.initial_pose_publisher_initialized = false;
  }

  if (g_amr_mqtt_bridge_ros_state.navigate_status_message_initialized) {
    action_msgs__msg__GoalStatusArray__fini(&g_amr_mqtt_bridge_ros_state.navigate_status_message);
    g_amr_mqtt_bridge_ros_state.navigate_status_message_initialized = false;
  }
}

static void amr_mqtt_bridge_poll_navigate_action(void)
{
  rcl_ret_t rc;
  bool is_feedback_ready = false;
  bool is_status_ready = false;
  bool is_goal_response_ready = false;
  bool is_cancel_response_ready = false;
  bool is_result_response_ready = false;

  if (!g_amr_mqtt_bridge_ros_state.navigate_to_pose_client_initialized ||
    !g_amr_mqtt_bridge_ros_state.navigate_wait_set_initialized ||
    !g_amr_mqtt_bridge_ros_state.navigate_state.active)
  {
    return;
  }

  rc = rcl_wait_set_clear(&g_amr_mqtt_bridge_ros_state.navigate_wait_set);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return;
  }
  rc = rcl_action_wait_set_add_action_client(
    &g_amr_mqtt_bridge_ros_state.navigate_wait_set,
    &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
    NULL,
    NULL);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return;
  }
  rc = rcl_wait(&g_amr_mqtt_bridge_ros_state.navigate_wait_set, 0);
  if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
    rcl_reset_error();
    return;
  }
  if (rc == RCL_RET_TIMEOUT) {
    return;
  }

  rc = rcl_action_client_wait_set_get_entities_ready(
    &g_amr_mqtt_bridge_ros_state.navigate_wait_set,
    &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
    &is_feedback_ready,
    &is_status_ready,
    &is_goal_response_ready,
    &is_cancel_response_ready,
    &is_result_response_ready);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return;
  }
  (void)is_cancel_response_ready;

  if (is_goal_response_ready && g_amr_mqtt_bridge_ros_state.navigate_state.goal_response_pending) {
    rmw_request_id_t response_header;
    amr_msgs__action__NavigateToPose_SendGoal_Response goal_response;
    amr_msgs__action__NavigateToPose_GetResult_Request result_request;
    int64_t result_sequence_number = 0;
    char * response_payload = NULL;

    memset(&response_header, 0, sizeof(response_header));
    memset(&goal_response, 0, sizeof(goal_response));
    rc = rcl_action_take_goal_response(
      &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
      &response_header,
      &goal_response);
    if (rc == RCL_RET_OK &&
      response_header.sequence_number == g_amr_mqtt_bridge_ros_state.navigate_state.goal_request_sequence_number)
    {
      response_payload = amr_mqtt_bridge_serialize_navigate_goal_response(
        g_amr_mqtt_bridge_ros_state.navigate_state.request_id,
        g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid,
        &goal_response,
        goal_response.accepted ? "goal accepted" : "goal rejected");
      if (response_payload != NULL) {
        (void)amr_mqtt_bridge_publish_payload(
          g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
          response_payload,
          g_amr_mqtt_bridge_config.mqtt.command_qos);
        free(response_payload);
      }

      g_amr_mqtt_bridge_ros_state.navigate_state.goal_response_pending = false;
      if (!goal_response.accepted) {
        memset(&g_amr_mqtt_bridge_ros_state.navigate_state, 0, sizeof(g_amr_mqtt_bridge_ros_state.navigate_state));
      } else {
        memset(&result_request, 0, sizeof(result_request));
        memcpy(result_request.goal_id.uuid, g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid, 16U);
        rc = rcl_action_send_result_request(
          &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
          &result_request,
          &result_sequence_number);
        if (rc == RCL_RET_OK) {
          g_amr_mqtt_bridge_ros_state.navigate_state.result_response_pending = true;
          g_amr_mqtt_bridge_ros_state.navigate_state.result_request_sequence_number = result_sequence_number;
        } else {
          rcl_reset_error();
          memset(&g_amr_mqtt_bridge_ros_state.navigate_state, 0, sizeof(g_amr_mqtt_bridge_ros_state.navigate_state));
        }
      }
    }
  }

  if (is_feedback_ready) {
    amr_msgs__action__NavigateToPose_FeedbackMessage feedback_message;
    char * feedback_payload = NULL;

    memset(&feedback_message, 0, sizeof(feedback_message));
    if (amr_msgs__action__NavigateToPose_FeedbackMessage__init(&feedback_message)) {
      rc = rcl_action_take_feedback(
        &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
        &feedback_message);
      if (rc == RCL_RET_OK &&
        amr_mqtt_bridge_uuid_equals(
          feedback_message.goal_id.uuid,
          g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid))
      {
        feedback_payload = amr_mqtt_bridge_serialize_navigate_feedback_message(
          g_amr_mqtt_bridge_ros_state.navigate_state.request_id,
          &feedback_message);
        if (feedback_payload != NULL) {
          (void)amr_mqtt_bridge_publish_payload(
            g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose,
            feedback_payload,
            g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
          free(feedback_payload);
        }
      }
      amr_msgs__action__NavigateToPose_FeedbackMessage__fini(&feedback_message);
    }
  }

  if (is_status_ready) {
    rc = rcl_action_take_status(
      &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
      &g_amr_mqtt_bridge_ros_state.navigate_status_message);
    if (rc == RCL_RET_OK) {
      action_msgs__msg__GoalStatusArray__fini(&g_amr_mqtt_bridge_ros_state.navigate_status_message);
      g_amr_mqtt_bridge_ros_state.navigate_status_message_initialized =
        action_msgs__msg__GoalStatusArray__init(&g_amr_mqtt_bridge_ros_state.navigate_status_message);
    }
  }

  if (is_result_response_ready && g_amr_mqtt_bridge_ros_state.navigate_state.result_response_pending) {
    rmw_request_id_t response_header;
    amr_msgs__action__NavigateToPose_GetResult_Response result_response;
    char * result_payload = NULL;

    memset(&response_header, 0, sizeof(response_header));
    memset(&result_response, 0, sizeof(result_response));
    if (amr_msgs__action__NavigateToPose_GetResult_Response__init(&result_response)) {
      rc = rcl_action_take_result_response(
        &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
        &response_header,
        &result_response);
      if (rc == RCL_RET_OK &&
        response_header.sequence_number == g_amr_mqtt_bridge_ros_state.navigate_state.result_request_sequence_number)
      {
        result_payload = amr_mqtt_bridge_serialize_navigate_result_response(
          g_amr_mqtt_bridge_ros_state.navigate_state.request_id,
          g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid,
          &result_response);
        if (result_payload != NULL) {
          (void)amr_mqtt_bridge_publish_payload(
            g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
            result_payload,
            g_amr_mqtt_bridge_config.mqtt.command_qos);
          free(result_payload);
        }
        memset(&g_amr_mqtt_bridge_ros_state.navigate_state, 0, sizeof(g_amr_mqtt_bridge_ros_state.navigate_state));
      }
      amr_msgs__action__NavigateToPose_GetResult_Response__fini(&result_response);
    }
  }
}

static void amr_mqtt_bridge_telemetry_callback(const void * message, void * context)
{
  const amr_mqtt_bridge_telemetry_endpoint_t * endpoint =
    (const amr_mqtt_bridge_telemetry_endpoint_t *)context;
  if (message == NULL || endpoint == NULL || endpoint->serializer == NULL) {
    return;
  }

  char * payload = endpoint->serializer(message);
  if (payload == NULL) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Failed to serialize '%s' telemetry payload",
      endpoint->label);
    return;
  }

  (void)amr_mqtt_bridge_publish_payload(
    endpoint->mqtt_topic,
    payload,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  free(payload);
}

static int amr_mqtt_bridge_init_messages(void)
{
  memset(&g_amr_mqtt_bridge_ros_state, 0, sizeof(g_amr_mqtt_bridge_ros_state));

  if (!geometry_msgs__msg__PoseStamped__init(&g_amr_mqtt_bridge_ros_state.robot_pose_message) ||
    !nav_msgs__msg__Path__init(&g_amr_mqtt_bridge_ros_state.global_path_message) ||
    !nav_msgs__msg__Path__init(&g_amr_mqtt_bridge_ros_state.local_path_message) ||
    !nav_msgs__msg__OccupancyGrid__init(&g_amr_mqtt_bridge_ros_state.map_message) ||
    !nav_msgs__msg__OccupancyGrid__init(&g_amr_mqtt_bridge_ros_state.global_costmap_message) ||
    !nav_msgs__msg__OccupancyGrid__init(&g_amr_mqtt_bridge_ros_state.local_costmap_message) ||
    !amr_msgs__msg__MotionStatus__init(&g_amr_mqtt_bridge_ros_state.motion_status_message) ||
    !amr_msgs__msg__ObstacleReport__init(&g_amr_mqtt_bridge_ros_state.obstacle_report_message))
  {
    RCUTILS_LOG_ERROR_NAMED("amr_mqtt_bridge", "Failed to initialize telemetry messages");
    amr_mqtt_bridge_fini_messages();
    return 1;
  }

  g_amr_mqtt_bridge_ros_state.messages_initialized = true;
  return 0;
}

static void amr_mqtt_bridge_fini_messages(void)
{
  if (!g_amr_mqtt_bridge_ros_state.messages_initialized) {
    return;
  }

  geometry_msgs__msg__PoseStamped__fini(&g_amr_mqtt_bridge_ros_state.robot_pose_message);
  nav_msgs__msg__Path__fini(&g_amr_mqtt_bridge_ros_state.global_path_message);
  nav_msgs__msg__Path__fini(&g_amr_mqtt_bridge_ros_state.local_path_message);
  nav_msgs__msg__OccupancyGrid__fini(&g_amr_mqtt_bridge_ros_state.map_message);
  nav_msgs__msg__OccupancyGrid__fini(&g_amr_mqtt_bridge_ros_state.global_costmap_message);
  nav_msgs__msg__OccupancyGrid__fini(&g_amr_mqtt_bridge_ros_state.local_costmap_message);
  amr_msgs__msg__MotionStatus__fini(&g_amr_mqtt_bridge_ros_state.motion_status_message);
  amr_msgs__msg__ObstacleReport__fini(&g_amr_mqtt_bridge_ros_state.obstacle_report_message);
  g_amr_mqtt_bridge_ros_state.messages_initialized = false;
}

static void amr_mqtt_bridge_configure_endpoint(
  amr_mqtt_bridge_telemetry_endpoint_t * endpoint,
  const char * label,
  const char * ros_topic,
  const char * mqtt_topic,
  rcl_subscription_t * subscription,
  void * message,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile,
  amr_mqtt_bridge_serializer_fn_t serializer)
{
  endpoint->label = label;
  endpoint->ros_topic = ros_topic;
  endpoint->mqtt_topic = mqtt_topic;
  endpoint->subscription = subscription;
  endpoint->message = message;
  endpoint->type_support = type_support;
  endpoint->qos_profile = qos_profile;
  endpoint->serializer = serializer;
}

static int amr_mqtt_bridge_add_subscription(amr_mqtt_bridge_telemetry_endpoint_t * endpoint)
{
  rcl_ret_t rc = rclc_subscription_init(
    endpoint->subscription,
    &g_amr_mqtt_bridge_runtime.node,
    endpoint->type_support,
    endpoint->ros_topic,
    endpoint->qos_profile);
  if (rc != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Failed to create subscription for '%s' on '%s': %s",
      endpoint->label,
      endpoint->ros_topic,
      rcl_get_error_string().str);
    rcl_reset_error();
    return 1;
  }

  rc = rclc_executor_add_subscription_with_context(
    &g_amr_mqtt_bridge_runtime.executor,
    endpoint->subscription,
    endpoint->message,
    amr_mqtt_bridge_telemetry_callback,
    endpoint,
    ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Failed to add executor subscription for '%s': %s",
      endpoint->label,
      rcl_get_error_string().str);
    rcl_reset_error();
    return 1;
  }

  return 0;
}

static int amr_mqtt_bridge_init_subscriptions(void)
{
  g_amr_mqtt_bridge_ros_state.robot_pose_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.global_path_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.local_path_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.map_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.global_costmap_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.local_costmap_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.motion_status_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.obstacle_report_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.telemetry_endpoint_count = AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS;
  g_amr_mqtt_bridge_ros_state.subscriptions_initialized = true;

  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[0],
    "robot_pose",
    g_amr_mqtt_bridge_config.ros.topic_robot_pose,
    g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose,
    &g_amr_mqtt_bridge_ros_state.robot_pose_subscription,
    &g_amr_mqtt_bridge_ros_state.robot_pose_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, geometry_msgs, msg, PoseStamped)(),
    &k_default_qos,
    amr_mqtt_bridge_serialize_pose_stamped);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[1],
    "global_path",
    g_amr_mqtt_bridge_config.ros.topic_global_path,
    g_amr_mqtt_bridge_config.mqtt.telemetry_global_path,
    &g_amr_mqtt_bridge_ros_state.global_path_subscription,
    &g_amr_mqtt_bridge_ros_state.global_path_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, Path)(),
    &k_default_qos,
    amr_mqtt_bridge_serialize_path);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[2],
    "local_path",
    g_amr_mqtt_bridge_config.ros.topic_local_path,
    g_amr_mqtt_bridge_config.mqtt.telemetry_local_path,
    &g_amr_mqtt_bridge_ros_state.local_path_subscription,
    &g_amr_mqtt_bridge_ros_state.local_path_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, Path)(),
    &k_default_qos,
    amr_mqtt_bridge_serialize_path);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[3],
    "map",
    g_amr_mqtt_bridge_config.ros.topic_map,
    g_amr_mqtt_bridge_config.mqtt.telemetry_map,
    &g_amr_mqtt_bridge_ros_state.map_subscription,
    &g_amr_mqtt_bridge_ros_state.map_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, OccupancyGrid)(),
    &k_transient_local_qos,
    amr_mqtt_bridge_serialize_occupancy_grid);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[4],
    "global_costmap",
    g_amr_mqtt_bridge_config.ros.topic_global_costmap,
    g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap,
    &g_amr_mqtt_bridge_ros_state.global_costmap_subscription,
    &g_amr_mqtt_bridge_ros_state.global_costmap_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, OccupancyGrid)(),
    &k_default_qos,
    amr_mqtt_bridge_serialize_occupancy_grid);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[5],
    "local_costmap",
    g_amr_mqtt_bridge_config.ros.topic_local_costmap,
    g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap,
    &g_amr_mqtt_bridge_ros_state.local_costmap_subscription,
    &g_amr_mqtt_bridge_ros_state.local_costmap_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, OccupancyGrid)(),
    &k_default_qos,
    amr_mqtt_bridge_serialize_occupancy_grid);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[6],
    "motion_status",
    g_amr_mqtt_bridge_config.ros.topic_motion_status,
    g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status,
    &g_amr_mqtt_bridge_ros_state.motion_status_subscription,
    &g_amr_mqtt_bridge_ros_state.motion_status_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, amr_msgs, msg, MotionStatus)(),
    &k_default_qos,
    amr_mqtt_bridge_serialize_motion_status);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[7],
    "obstacle_report",
    g_amr_mqtt_bridge_config.ros.topic_obstacle_report,
    g_amr_mqtt_bridge_config.mqtt.telemetry_obstacle_report,
    &g_amr_mqtt_bridge_ros_state.obstacle_report_subscription,
    &g_amr_mqtt_bridge_ros_state.obstacle_report_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, amr_msgs, msg, ObstacleReport)(),
    &k_default_qos,
    amr_mqtt_bridge_serialize_obstacle_report);

  for (size_t index = 0; index < g_amr_mqtt_bridge_ros_state.telemetry_endpoint_count; ++index) {
    if (amr_mqtt_bridge_add_subscription(&g_amr_mqtt_bridge_ros_state.telemetry_endpoints[index]) != 0) {
      amr_mqtt_bridge_fini_subscriptions();
      return 1;
    }
  }

  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_bridge",
    "Initialized %zu telemetry subscriptions",
    g_amr_mqtt_bridge_ros_state.telemetry_endpoint_count);
  return 0;
}

static void amr_mqtt_bridge_fini_subscriptions(void)
{
  if (!g_amr_mqtt_bridge_ros_state.subscriptions_initialized) {
    return;
  }

  for (size_t index = 0; index < g_amr_mqtt_bridge_ros_state.telemetry_endpoint_count; ++index) {
    rcl_subscription_t * subscription = g_amr_mqtt_bridge_ros_state.telemetry_endpoints[index].subscription;
    if (subscription != NULL) {
      rcl_ret_t rc = rcl_subscription_fini(subscription, &g_amr_mqtt_bridge_runtime.node);
      amr_mqtt_bridge_log_rcl_fini_error("subscription", rc);
    }
  }

  g_amr_mqtt_bridge_ros_state.subscriptions_initialized = false;
}

int amr_mqtt_bridge_init(int argc, const char * const * argv)
{
  rcutils_ret_t logging_ret = rcutils_logging_set_logger_level(
    "amr_mqtt_bridge", RCUTILS_LOG_SEVERITY_INFO);
  if (logging_ret != RCUTILS_RET_OK) {
    fprintf(stderr, "Failed to configure logger level: %s\n", rcutils_get_error_string().str);
    rcutils_reset_error();
  }

  g_amr_mqtt_bridge_runtime.allocator = rcl_get_default_allocator();
  (void)memset(&g_amr_mqtt_bridge_runtime.support, 0, sizeof(g_amr_mqtt_bridge_runtime.support));
  g_amr_mqtt_bridge_runtime.node = rcl_get_zero_initialized_node();
  g_amr_mqtt_bridge_runtime.executor = rclc_executor_get_zero_initialized_executor();
  g_amr_mqtt_bridge_runtime.return_code = 0;
  amr_mqtt_bridge_set_default_config();

  rcl_ret_t rc = rclc_support_init(
    &g_amr_mqtt_bridge_runtime.support,
    argc,
    argv,
    &g_amr_mqtt_bridge_runtime.allocator);
  if (rc != RCL_RET_OK) {
    fprintf(stderr, "Failed to initialize rclc support: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    return 1;
  }

  amr_mqtt_bridge_load_parameter_overrides();
  rc = rclc_node_init_default(
    &g_amr_mqtt_bridge_runtime.node,
    "mqtt_bridge",
    "/amr",
    &g_amr_mqtt_bridge_runtime.support);
  if (rc != RCL_RET_OK) {
    fprintf(stderr, "Failed to initialize amr_mqtt_bridge node: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    amr_mqtt_bridge_log_rcl_fini_error(
      "support", rclc_support_fini(&g_amr_mqtt_bridge_runtime.support));
    return 1;
  }

  rc = rclc_executor_init(
    &g_amr_mqtt_bridge_runtime.executor,
    &g_amr_mqtt_bridge_runtime.support.context,
    AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS,
    &g_amr_mqtt_bridge_runtime.allocator);
  if (rc != RCL_RET_OK) {
    fprintf(stderr, "Failed to initialize executor: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    amr_mqtt_bridge_log_rcl_fini_error(
      "node", rcl_node_fini(&g_amr_mqtt_bridge_runtime.node));
    amr_mqtt_bridge_log_rcl_fini_error(
      "support", rclc_support_fini(&g_amr_mqtt_bridge_runtime.support));
    return 1;
  }

  if (amr_mqtt_bridge_connect_mqtt() != 0) {
    amr_mqtt_bridge_log_rcl_fini_error(
      "executor", rclc_executor_fini(&g_amr_mqtt_bridge_runtime.executor));
    amr_mqtt_bridge_log_rcl_fini_error(
      "node", rcl_node_fini(&g_amr_mqtt_bridge_runtime.node));
    amr_mqtt_bridge_log_rcl_fini_error(
      "support", rclc_support_fini(&g_amr_mqtt_bridge_runtime.support));
    return 1;
  }

  if (amr_mqtt_bridge_init_messages() != 0 ||
    amr_mqtt_bridge_init_subscriptions() != 0 ||
    amr_mqtt_bridge_init_command_interfaces() != 0)
  {
    amr_mqtt_bridge_disconnect_mqtt();
    amr_mqtt_bridge_fini_command_interfaces();
    amr_mqtt_bridge_fini_subscriptions();
    amr_mqtt_bridge_fini_messages();
    amr_mqtt_bridge_log_rcl_fini_error(
      "executor", rclc_executor_fini(&g_amr_mqtt_bridge_runtime.executor));
    amr_mqtt_bridge_log_rcl_fini_error(
      "node", rcl_node_fini(&g_amr_mqtt_bridge_runtime.node));
    amr_mqtt_bridge_log_rcl_fini_error(
      "support", rclc_support_fini(&g_amr_mqtt_bridge_runtime.support));
    return 1;
  }

  g_amr_mqtt_bridge_runtime.is_initialized = true;
  amr_mqtt_bridge_log_config();
  return 0;
}

void amr_mqtt_bridge_spin(void)
{
  if (!g_amr_mqtt_bridge_runtime.is_initialized) {
    return;
  }

  while (rcl_context_is_valid(&g_amr_mqtt_bridge_runtime.support.context)) {
    rcl_ret_t rc = rclc_executor_spin_some(
      &g_amr_mqtt_bridge_runtime.executor,
      20 * 1000 * 1000);
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
      fprintf(stderr, "Executor error: %s\n", rcl_get_error_string().str);
      rcl_reset_error();
      g_amr_mqtt_bridge_runtime.return_code = 1;
      break;
    }
    amr_mqtt_bridge_poll_mqtt();
    amr_mqtt_bridge_poll_navigate_action();
  }
}

int amr_mqtt_bridge_shutdown(void)
{
  if (!g_amr_mqtt_bridge_runtime.is_initialized) {
    return g_amr_mqtt_bridge_runtime.return_code;
  }

  amr_mqtt_bridge_fini_command_interfaces();
  amr_mqtt_bridge_fini_subscriptions();
  amr_mqtt_bridge_fini_messages();
  amr_mqtt_bridge_disconnect_mqtt();

  rcl_ret_t rc = rclc_executor_fini(&g_amr_mqtt_bridge_runtime.executor);
  amr_mqtt_bridge_log_rcl_fini_error("executor", rc);

  rc = rcl_node_fini(&g_amr_mqtt_bridge_runtime.node);
  amr_mqtt_bridge_log_rcl_fini_error("node", rc);

  rc = rclc_support_fini(&g_amr_mqtt_bridge_runtime.support);
  amr_mqtt_bridge_log_rcl_fini_error("support", rc);

  g_amr_mqtt_bridge_runtime.is_initialized = false;
  return g_amr_mqtt_bridge_runtime.return_code;
}
