#include "amr_mqtt_bridge/node.h"

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

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

static const char * k_node_name = "/amr/mqtt_bridge";
static const rmw_qos_profile_t k_sensor_qos = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};
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

static bool amr_mqtt_bridge_extract_json_double_in_range(
  const char * begin,
  const char * end,
  const char * key,
  double * output);

static bool amr_mqtt_bridge_extract_json_object_in_range(
  const char * begin,
  const char * end,
  const char * key,
  const char ** object_begin,
  const char ** object_end);

static bool amr_mqtt_bridge_extract_json_string_in_range(
  const char * begin,
  const char * end,
  const char * key,
  char * output,
  size_t output_capacity);

static void amr_mqtt_bridge_publish_simple_response(
  const char * mqtt_topic,
  const char * request_id,
  bool success,
  const char * message);

static void amr_mqtt_bridge_log_rcl_error(const char * label, rcl_ret_t rc)
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

static uint64_t amr_mqtt_bridge_now_ms(void)
{
  struct timespec timestamp;
  if (clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0) {
    return 0U;
  }

  return ((uint64_t)timestamp.tv_sec * 1000ULL) + ((uint64_t)timestamp.tv_nsec / 1000000ULL);
}

static void amr_mqtt_bridge_set_default_config(void)
{
  memset(&g_amr_mqtt_bridge_config, 0, sizeof(g_amr_mqtt_bridge_config));

  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.broker.host,
    sizeof(g_amr_mqtt_bridge_config.broker.host),
    "192.168.61.35");
  g_amr_mqtt_bridge_config.broker.port = 1883;
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.broker.client_id,
    sizeof(g_amr_mqtt_bridge_config.broker.client_id),
    "amr_mqtt_bridge");
  g_amr_mqtt_bridge_config.broker.keep_alive_sec = 300;
  g_amr_mqtt_bridge_config.broker.clean_session = false;

  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.root,
    sizeof(g_amr_mqtt_bridge_config.mqtt.root),
    "amr/robot/turtlebot3");
  g_amr_mqtt_bridge_config.mqtt.telemetry_qos = 0;
  g_amr_mqtt_bridge_config.mqtt.command_qos = 0;
  g_amr_mqtt_bridge_config.mqtt.service_qos = 0;
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_map,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_map),
    "amr/robot/turtlebot3/telemetry/map");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose),
    "amr/robot/turtlebot3/telemetry/robot_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_global_path,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_global_path),
    "amr/robot/turtlebot3/telemetry/global_path");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_local_path,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_local_path),
    "amr/robot/turtlebot3/telemetry/local_path");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap),
    "amr/robot/turtlebot3/telemetry/global_costmap");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap),
    "amr/robot/turtlebot3/telemetry/local_costmap");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status),
    "amr/robot/turtlebot3/telemetry/motion_status");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_scan,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_scan),
    "amr/robot/turtlebot3/telemetry/scan");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_odom,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_odom),
    "amr/robot/turtlebot3/telemetry/odom");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_imu,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_imu),
    "amr/robot/turtlebot3/telemetry/imu");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_tf,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_tf),
    "amr/robot/turtlebot3/telemetry/tf");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_tf_static,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_tf_static),
    "amr/robot/turtlebot3/telemetry/tf_static");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_joint_states,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_joint_states),
    "amr/robot/turtlebot3/telemetry/joint_states");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_robot_description,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_robot_description),
    "amr/robot/turtlebot3/telemetry/robot_description");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_battery_state,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_battery_state),
    "amr/robot/turtlebot3/telemetry/battery_state");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_temp_map,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_temp_map),
    "amr/robot/turtlebot3/telemetry/temp_map");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_mapping_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_mapping_pose),
    "amr/robot/turtlebot3/telemetry/mapping_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.telemetry_slam_graph,
    sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_slam_graph),
    "amr/robot/turtlebot3/telemetry/slam_graph");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.command_cmd_vel,
    sizeof(g_amr_mqtt_bridge_config.mqtt.command_cmd_vel),
    "amr/robot/turtlebot3/command/cmd_vel");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.command_save_map,
    sizeof(g_amr_mqtt_bridge_config.mqtt.command_save_map),
    "amr/command/save_map");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose),
    "amr/command/set_initial_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose),
    "amr/command/navigate_to_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.command_cancel_navigate_to_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.command_cancel_navigate_to_pose),
    "amr/command/cancel_navigate_to_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.command_ping,
    sizeof(g_amr_mqtt_bridge_config.mqtt.command_ping),
    "amr/command/ping");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose),
    "amr/feedback/navigate_to_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.status_navigate_to_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.status_navigate_to_pose),
    "amr/status/navigate_to_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose),
    "amr/response/set_initial_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
    sizeof(g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose),
    "amr/response/navigate_to_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.response_save_map,
    sizeof(g_amr_mqtt_bridge_config.mqtt.response_save_map),
    "amr/response/save_map");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.response_ping,
    sizeof(g_amr_mqtt_bridge_config.mqtt.response_ping),
    "amr/response/ping");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.request_plan_segment,
    sizeof(g_amr_mqtt_bridge_config.mqtt.request_plan_segment),
    "amr/request/plan_segment");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.request_plan_route,
    sizeof(g_amr_mqtt_bridge_config.mqtt.request_plan_route),
    "amr/request/plan_route");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
    sizeof(g_amr_mqtt_bridge_config.mqtt.response_plan_segment),
    "amr/response/plan_segment");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.mqtt.response_plan_route,
    sizeof(g_amr_mqtt_bridge_config.mqtt.response_plan_route),
    "amr/response/plan_route");

  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_map,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_map),
    "/amr/map/data");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_robot_pose,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_robot_pose),
    "/amr/localization/pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_global_path,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_global_path),
    "/amr/planner/global");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_local_path,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_local_path),
    "/amr/planner/local");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_global_costmap,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_global_costmap),
    "/amr/costmap/global");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_local_costmap,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_local_costmap),
    "/amr/costmap/local");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_motion_status,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_motion_status),
    "/amr/motion/status");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_scan,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_scan),
    "/scan");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_odom,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_odom),
    "/odom");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_imu,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_imu),
    "/imu");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_tf,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_tf),
    "/tf");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_tf_static,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_tf_static),
    "/tf_static");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_joint_states,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_joint_states),
    "/joint_states");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_robot_description,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_robot_description),
    "/robot_description");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_battery_state,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_battery_state),
    "/battery_state");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_temp_map,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_temp_map),
    "/amr/map/temp");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_mapping_pose,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_mapping_pose),
    "/amr/slam_mapper/pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_slam_graph,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_slam_graph),
    "/amr/slam_mapper/graph_debug");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_cmd_vel,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_cmd_vel),
    "/cmd_vel");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_initial_pose,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_initial_pose),
    "/amr/localization/initial_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_navigate_feedback,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_navigate_feedback),
    "/amr/navigator/navigate_to_pose/feedback");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.topic_navigate_status,
    sizeof(g_amr_mqtt_bridge_config.ros.topic_navigate_status),
    "/amr/navigator/navigate_to_pose/status");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.service_plan_segment,
    sizeof(g_amr_mqtt_bridge_config.ros.service_plan_segment),
    "/amr/global_planner/plan_segment");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.service_plan_route,
    sizeof(g_amr_mqtt_bridge_config.ros.service_plan_route),
    "/amr/global_planner/plan_route");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.action_navigate_to_pose,
    sizeof(g_amr_mqtt_bridge_config.ros.action_navigate_to_pose),
    "/amr/navigator/navigate_to_pose");
  amr_mqtt_bridge_copy_string(
    g_amr_mqtt_bridge_config.ros.save_directory,
    sizeof(g_amr_mqtt_bridge_config.ros.save_directory),
    "/home/burger1/ws/data/maps");
  g_amr_mqtt_bridge_config.footprint_polygon_size = 0U;
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
    if (strcmp(current_name, node_name) == 0 ||
      (node_name[0] == '/' && strcmp(current_name, node_name + 1) == 0) ||
      (current_name[0] == '/' && strcmp(current_name + 1, node_name) == 0))
    {
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
  const rcl_variant_t * variant =
    amr_mqtt_bridge_find_param_variant(node_params, parameter_name);
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
  const rcl_variant_t * variant =
    amr_mqtt_bridge_find_param_variant(node_params, parameter_name);
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
  const rcl_variant_t * variant =
    amr_mqtt_bridge_find_param_variant(node_params, parameter_name);
  if (variant == NULL || variant->bool_value == NULL || destination == NULL) {
    return;
  }
  *destination = *variant->bool_value;
}

static void amr_mqtt_bridge_read_double_array_param(
  const rcl_node_params_t * node_params,
  const char * parameter_name,
  double * destination,
  size_t destination_capacity,
  size_t * destination_size)
{
  const rcl_variant_t * variant =
    amr_mqtt_bridge_find_param_variant(node_params, parameter_name);
  size_t value_count = 0U;

  if (destination == NULL || destination_size == NULL || destination_capacity == 0U || variant == NULL) {
    return;
  }

  if (variant->double_array_value != NULL) {
    value_count = variant->double_array_value->size;
    if (value_count > destination_capacity) {
      value_count = destination_capacity;
    }
    for (size_t index = 0; index < value_count; ++index) {
      destination[index] = variant->double_array_value->values[index];
    }
    *destination_size = value_count;
    return;
  }

  if (variant->integer_array_value != NULL) {
    value_count = variant->integer_array_value->size;
    if (value_count > destination_capacity) {
      value_count = destination_capacity;
    }
    for (size_t index = 0; index < value_count; ++index) {
      destination[index] = (double)variant->integer_array_value->values[index];
    }
    *destination_size = value_count;
  }
}

static void amr_mqtt_bridge_load_parameter_overrides(void)
{
  rcl_params_t * parameter_overrides = NULL;
  const rcl_node_params_t * node_params = NULL;
  rcl_ret_t rc = rcl_arguments_get_param_overrides(
    &g_amr_mqtt_bridge_runtime.support.context.global_arguments,
    &parameter_overrides);
  if (rc != RCL_RET_OK || parameter_overrides == NULL) {
    if (rc != RCL_RET_OK) {
      rcl_reset_error();
    }
    return;
  }

  node_params = amr_mqtt_bridge_find_node_params(parameter_overrides, k_node_name);
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
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.map", g_amr_mqtt_bridge_config.mqtt.telemetry_map, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_map));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.robot_pose", g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.global_path", g_amr_mqtt_bridge_config.mqtt.telemetry_global_path, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_global_path));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.local_path", g_amr_mqtt_bridge_config.mqtt.telemetry_local_path, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_local_path));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.global_costmap", g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.local_costmap", g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.motion_status", g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.scan", g_amr_mqtt_bridge_config.mqtt.telemetry_scan, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_scan));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.odom", g_amr_mqtt_bridge_config.mqtt.telemetry_odom, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_odom));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.imu", g_amr_mqtt_bridge_config.mqtt.telemetry_imu, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_imu));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.tf", g_amr_mqtt_bridge_config.mqtt.telemetry_tf, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_tf));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.tf_static", g_amr_mqtt_bridge_config.mqtt.telemetry_tf_static, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_tf_static));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.joint_states", g_amr_mqtt_bridge_config.mqtt.telemetry_joint_states, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_joint_states));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.robot_description", g_amr_mqtt_bridge_config.mqtt.telemetry_robot_description, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_robot_description));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.battery_state", g_amr_mqtt_bridge_config.mqtt.telemetry_battery_state, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_battery_state));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.temp_map", g_amr_mqtt_bridge_config.mqtt.telemetry_temp_map, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_temp_map));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.mapping_pose", g_amr_mqtt_bridge_config.mqtt.telemetry_mapping_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_mapping_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.telemetry.slam_graph", g_amr_mqtt_bridge_config.mqtt.telemetry_slam_graph, sizeof(g_amr_mqtt_bridge_config.mqtt.telemetry_slam_graph));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.command.cmd_vel", g_amr_mqtt_bridge_config.mqtt.command_cmd_vel, sizeof(g_amr_mqtt_bridge_config.mqtt.command_cmd_vel));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.command.save_map", g_amr_mqtt_bridge_config.mqtt.command_save_map, sizeof(g_amr_mqtt_bridge_config.mqtt.command_save_map));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.command.set_initial_pose", g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.command.navigate_to_pose", g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.command.cancel_navigate_to_pose", g_amr_mqtt_bridge_config.mqtt.command_cancel_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.command_cancel_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.command.ping", g_amr_mqtt_bridge_config.mqtt.command_ping, sizeof(g_amr_mqtt_bridge_config.mqtt.command_ping));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.feedback.navigate_to_pose", g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.status.navigate_to_pose", g_amr_mqtt_bridge_config.mqtt.status_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.status_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.set_initial_pose", g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.response_set_initial_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.navigate_to_pose", g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.save_map", g_amr_mqtt_bridge_config.mqtt.response_save_map, sizeof(g_amr_mqtt_bridge_config.mqtt.response_save_map));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.ping", g_amr_mqtt_bridge_config.mqtt.response_ping, sizeof(g_amr_mqtt_bridge_config.mqtt.response_ping));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.request.plan_segment", g_amr_mqtt_bridge_config.mqtt.request_plan_segment, sizeof(g_amr_mqtt_bridge_config.mqtt.request_plan_segment));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.request.plan_route", g_amr_mqtt_bridge_config.mqtt.request_plan_route, sizeof(g_amr_mqtt_bridge_config.mqtt.request_plan_route));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.plan_segment", g_amr_mqtt_bridge_config.mqtt.response_plan_segment, sizeof(g_amr_mqtt_bridge_config.mqtt.response_plan_segment));
    amr_mqtt_bridge_read_string_param(node_params, "mqtt.topics.response.plan_route", g_amr_mqtt_bridge_config.mqtt.response_plan_route, sizeof(g_amr_mqtt_bridge_config.mqtt.response_plan_route));

    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.map", g_amr_mqtt_bridge_config.ros.topic_map, sizeof(g_amr_mqtt_bridge_config.ros.topic_map));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.robot_pose", g_amr_mqtt_bridge_config.ros.topic_robot_pose, sizeof(g_amr_mqtt_bridge_config.ros.topic_robot_pose));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.global_path", g_amr_mqtt_bridge_config.ros.topic_global_path, sizeof(g_amr_mqtt_bridge_config.ros.topic_global_path));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.local_path", g_amr_mqtt_bridge_config.ros.topic_local_path, sizeof(g_amr_mqtt_bridge_config.ros.topic_local_path));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.global_costmap", g_amr_mqtt_bridge_config.ros.topic_global_costmap, sizeof(g_amr_mqtt_bridge_config.ros.topic_global_costmap));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.local_costmap", g_amr_mqtt_bridge_config.ros.topic_local_costmap, sizeof(g_amr_mqtt_bridge_config.ros.topic_local_costmap));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.motion_status", g_amr_mqtt_bridge_config.ros.topic_motion_status, sizeof(g_amr_mqtt_bridge_config.ros.topic_motion_status));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.scan", g_amr_mqtt_bridge_config.ros.topic_scan, sizeof(g_amr_mqtt_bridge_config.ros.topic_scan));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.odom", g_amr_mqtt_bridge_config.ros.topic_odom, sizeof(g_amr_mqtt_bridge_config.ros.topic_odom));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.imu", g_amr_mqtt_bridge_config.ros.topic_imu, sizeof(g_amr_mqtt_bridge_config.ros.topic_imu));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.tf", g_amr_mqtt_bridge_config.ros.topic_tf, sizeof(g_amr_mqtt_bridge_config.ros.topic_tf));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.tf_static", g_amr_mqtt_bridge_config.ros.topic_tf_static, sizeof(g_amr_mqtt_bridge_config.ros.topic_tf_static));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.joint_states", g_amr_mqtt_bridge_config.ros.topic_joint_states, sizeof(g_amr_mqtt_bridge_config.ros.topic_joint_states));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.robot_description", g_amr_mqtt_bridge_config.ros.topic_robot_description, sizeof(g_amr_mqtt_bridge_config.ros.topic_robot_description));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.battery_state", g_amr_mqtt_bridge_config.ros.topic_battery_state, sizeof(g_amr_mqtt_bridge_config.ros.topic_battery_state));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.temp_map", g_amr_mqtt_bridge_config.ros.topic_temp_map, sizeof(g_amr_mqtt_bridge_config.ros.topic_temp_map));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.mapping_pose", g_amr_mqtt_bridge_config.ros.topic_mapping_pose, sizeof(g_amr_mqtt_bridge_config.ros.topic_mapping_pose));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.slam_graph", g_amr_mqtt_bridge_config.ros.topic_slam_graph, sizeof(g_amr_mqtt_bridge_config.ros.topic_slam_graph));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.cmd_vel", g_amr_mqtt_bridge_config.ros.topic_cmd_vel, sizeof(g_amr_mqtt_bridge_config.ros.topic_cmd_vel));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.initial_pose", g_amr_mqtt_bridge_config.ros.topic_initial_pose, sizeof(g_amr_mqtt_bridge_config.ros.topic_initial_pose));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.navigate_feedback", g_amr_mqtt_bridge_config.ros.topic_navigate_feedback, sizeof(g_amr_mqtt_bridge_config.ros.topic_navigate_feedback));
    amr_mqtt_bridge_read_string_param(node_params, "ros.topics.navigate_status", g_amr_mqtt_bridge_config.ros.topic_navigate_status, sizeof(g_amr_mqtt_bridge_config.ros.topic_navigate_status));
    amr_mqtt_bridge_read_string_param(node_params, "ros.services.plan_segment", g_amr_mqtt_bridge_config.ros.service_plan_segment, sizeof(g_amr_mqtt_bridge_config.ros.service_plan_segment));
    amr_mqtt_bridge_read_string_param(node_params, "ros.services.plan_route", g_amr_mqtt_bridge_config.ros.service_plan_route, sizeof(g_amr_mqtt_bridge_config.ros.service_plan_route));
    amr_mqtt_bridge_read_string_param(node_params, "ros.actions.navigate_to_pose", g_amr_mqtt_bridge_config.ros.action_navigate_to_pose, sizeof(g_amr_mqtt_bridge_config.ros.action_navigate_to_pose));
    amr_mqtt_bridge_read_string_param(node_params, "ros.save.directory", g_amr_mqtt_bridge_config.ros.save_directory, sizeof(g_amr_mqtt_bridge_config.ros.save_directory));
    amr_mqtt_bridge_read_double_array_param(
      node_params,
      "footprint.polygon",
      g_amr_mqtt_bridge_config.footprint_polygon,
      AMR_MQTT_BRIDGE_MAX_FOOTPRINT_POLYGON_VALUES,
      &g_amr_mqtt_bridge_config.footprint_polygon_size);
  }

  rcl_yaml_node_struct_fini(parameter_overrides);
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
  int required = 0;

  va_start(args, format);
  va_copy(args_copy, args);
  required = vsnprintf(NULL, 0, format, args_copy);
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
    if (current == '\\') {
      if (!amr_mqtt_bridge_builder_append(builder, "\\\\")) {
        return false;
      }
    } else if (current == '"') {
      if (!amr_mqtt_bridge_builder_append(builder, "\\\"")) {
        return false;
      }
    } else {
      char buffer[2] = {current, '\0'};
      if (!amr_mqtt_bridge_builder_append(builder, buffer)) {
        return false;
      }
    }
  }
  return amr_mqtt_bridge_builder_append(builder, "\"");
}

static char * amr_mqtt_bridge_builder_take(
  amr_mqtt_bridge_string_builder_t * builder)
{
  char * data = builder->data;
  builder->data = NULL;
  builder->length = 0U;
  builder->capacity = 0U;
  return data;
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
  if (!amr_mqtt_bridge_builder_init(&builder, 768U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &status->header) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"command_id\":%u,\"active\":%s,\"goal_reached\":%s,\"obstacle_detected\":%s,"
      "\"blocked\":%s,\"stalled\":%s,\"local_plan_valid\":%s,\"costmap_blocked\":%s,"
      "\"safety_gate_blocked\":%s,\"has_blocked_pose\":%s,"
      "\"remaining_distance\":%.6f,\"heading_error\":%.6f,\"current_pose\":",
      status->command_id,
      status->active ? "true" : "false",
      status->goal_reached ? "true" : "false",
      status->obstacle_detected ? "true" : "false",
      status->blocked ? "true" : "false",
      status->stalled ? "true" : "false",
      status->local_plan_valid ? "true" : "false",
      status->costmap_blocked ? "true" : "false",
      status->safety_gate_blocked ? "true" : "false",
      status->has_blocked_pose ? "true" : "false",
      status->remaining_distance,
      status->heading_error) ||
    !amr_mqtt_bridge_append_pose_stamped(&builder, &status->current_pose) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"blocked_pose\":") ||
    !amr_mqtt_bridge_append_pose_stamped(&builder, &status->blocked_pose) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }
  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_scan(const void * message)
{
  const sensor_msgs__msg__LaserScan * scan = (const sensor_msgs__msg__LaserScan *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};

  if (!amr_mqtt_bridge_builder_init(&builder, 1024U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &scan->header) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"angle_min\":%.6f,\"angle_max\":%.6f,\"angle_increment\":%.6f,"
      "\"range_min\":%.6f,\"range_max\":%.6f,\"ranges_count\":%zu,\"ranges\":[",
      scan->angle_min,
      scan->angle_max,
      scan->angle_increment,
      scan->range_min,
      scan->range_max,
      scan->ranges.size))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  for (size_t index = 0; index < scan->ranges.size; ++index) {
    if (index > 0U && !amr_mqtt_bridge_builder_append(&builder, ",")) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    if (!isfinite(scan->ranges.data[index])) {
      if (!amr_mqtt_bridge_builder_append(&builder, "null")) {
        amr_mqtt_bridge_builder_fini(&builder);
        return NULL;
      }
      continue;
    }
    if (!amr_mqtt_bridge_builder_appendf(&builder, "%.6f", scan->ranges.data[index])) {
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

static char * amr_mqtt_bridge_serialize_odom(const void * message)
{
  const nav_msgs__msg__Odometry * odom = (const nav_msgs__msg__Odometry *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};

  if (!amr_mqtt_bridge_builder_init(&builder, 512U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &odom->header) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"child_frame_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, odom->child_frame_id.data) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"pose\":{\"position\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
      "\"orientation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f}},"
      "\"twist\":{\"linear\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
      "\"angular\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f}}}",
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z,
      odom->pose.pose.orientation.w,
      odom->twist.twist.linear.x,
      odom->twist.twist.linear.y,
      odom->twist.twist.linear.z,
      odom->twist.twist.angular.x,
      odom->twist.twist.angular.y,
      odom->twist.twist.angular.z))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_imu(const void * message)
{
  const sensor_msgs__msg__Imu * imu = (const sensor_msgs__msg__Imu *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};

  if (!amr_mqtt_bridge_builder_init(&builder, 512U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &imu->header) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"orientation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f},"
      "\"angular_velocity\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
      "\"linear_acceleration\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f}}",
      imu->orientation.x,
      imu->orientation.y,
      imu->orientation.z,
      imu->orientation.w,
      imu->angular_velocity.x,
      imu->angular_velocity.y,
      imu->angular_velocity.z,
      imu->linear_acceleration.x,
      imu->linear_acceleration.y,
      imu->linear_acceleration.z))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static bool amr_mqtt_bridge_builder_append_double_array(
  amr_mqtt_bridge_string_builder_t * builder,
  const double * values,
  size_t count)
{
  if (!amr_mqtt_bridge_builder_append(builder, "[")) {
    return false;
  }

  for (size_t index = 0; index < count; ++index) {
    if (index > 0U && !amr_mqtt_bridge_builder_append(builder, ",")) {
      return false;
    }
    if (!amr_mqtt_bridge_builder_appendf(builder, "%.6f", values[index])) {
      return false;
    }
  }

  return amr_mqtt_bridge_builder_append(builder, "]");
}

static bool amr_mqtt_bridge_builder_append_string_array(
  amr_mqtt_bridge_string_builder_t * builder,
  const rosidl_runtime_c__String__Sequence * values)
{
  if (!amr_mqtt_bridge_builder_append(builder, "[")) {
    return false;
  }

  for (size_t index = 0; index < values->size; ++index) {
    if (index > 0U && !amr_mqtt_bridge_builder_append(builder, ",")) {
      return false;
    }
    if (!amr_mqtt_bridge_builder_append_json_string(builder, values->data[index].data)) {
      return false;
    }
  }

  return amr_mqtt_bridge_builder_append(builder, "]");
}

static bool amr_mqtt_bridge_append_transform_stamped(
  amr_mqtt_bridge_string_builder_t * builder,
  const geometry_msgs__msg__TransformStamped * transform)
{
  const double yaw = amr_mqtt_bridge_quaternion_to_yaw(
    transform->transform.rotation.x,
    transform->transform.rotation.y,
    transform->transform.rotation.z,
    transform->transform.rotation.w);
  return amr_mqtt_bridge_builder_append(builder, "{") &&
    amr_mqtt_bridge_append_header(builder, &transform->header) &&
    amr_mqtt_bridge_builder_append(builder, ",\"child_frame_id\":") &&
    amr_mqtt_bridge_builder_append_json_string(builder, transform->child_frame_id.data) &&
    amr_mqtt_bridge_builder_appendf(
      builder,
      ",\"translation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
      "\"rotation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f,\"yaw\":%.6f}}",
      transform->transform.translation.x,
      transform->transform.translation.y,
      transform->transform.translation.z,
      transform->transform.rotation.x,
      transform->transform.rotation.y,
      transform->transform.rotation.z,
      transform->transform.rotation.w,
      yaw);
}

static char * amr_mqtt_bridge_serialize_tf_message(const void * message)
{
  const tf2_msgs__msg__TFMessage * tf_message = (const tf2_msgs__msg__TFMessage *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};

  if (!amr_mqtt_bridge_builder_init(&builder, 1024U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"transforms\":["))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  for (size_t index = 0; index < tf_message->transforms.size; ++index) {
    if (index > 0U && !amr_mqtt_bridge_builder_append(&builder, ",")) {
      amr_mqtt_bridge_builder_fini(&builder);
      return NULL;
    }
    if (!amr_mqtt_bridge_append_transform_stamped(&builder, &tf_message->transforms.data[index])) {
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

static bool amr_mqtt_bridge_build_viz_topic(
  const char * raw_topic,
  char * viz_topic,
  size_t viz_topic_capacity)
{
  const char * telemetry_marker = NULL;
  int written = 0;

  if (raw_topic == NULL || viz_topic == NULL || viz_topic_capacity == 0U) {
    return false;
  }

  telemetry_marker = strstr(raw_topic, "/telemetry/");
  if (telemetry_marker == NULL) {
    return false;
  }

  written = snprintf(
    viz_topic,
    viz_topic_capacity,
    "%.*s/viz/%s",
    (int)(telemetry_marker - raw_topic),
    raw_topic,
    telemetry_marker + strlen("/telemetry/"));
  return written >= 0 && (size_t)written < viz_topic_capacity;
}

static char * amr_mqtt_bridge_serialize_joint_states(const void * message)
{
  const sensor_msgs__msg__JointState * joint_states = (const sensor_msgs__msg__JointState *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};

  if (!amr_mqtt_bridge_builder_init(&builder, 512U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &joint_states->header) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"name\":") ||
    !amr_mqtt_bridge_builder_append_string_array(&builder, &joint_states->name) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"position\":") ||
    !amr_mqtt_bridge_builder_append_double_array(&builder, joint_states->position.data, joint_states->position.size) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"velocity\":") ||
    !amr_mqtt_bridge_builder_append_double_array(&builder, joint_states->velocity.data, joint_states->velocity.size) ||
    !amr_mqtt_bridge_builder_append(&builder, ",\"effort\":") ||
    !amr_mqtt_bridge_builder_append_double_array(&builder, joint_states->effort.data, joint_states->effort.size) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_string_message(const void * message)
{
  const std_msgs__msg__String * string_message = (const std_msgs__msg__String *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};

  if (!amr_mqtt_bridge_builder_init(&builder, 512U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"data\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, string_message->data.data))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  if (g_amr_mqtt_bridge_config.footprint_polygon_size > 0U &&
    (!amr_mqtt_bridge_builder_append(&builder, ",\"footprint_polygon\":") ||
    !amr_mqtt_bridge_builder_append_double_array(
      &builder,
      g_amr_mqtt_bridge_config.footprint_polygon,
      g_amr_mqtt_bridge_config.footprint_polygon_size)))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  if (!amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_string_json_message(const void * message)
{
  const std_msgs__msg__String * string_message = (const std_msgs__msg__String *)message;
  char * payload = NULL;
  size_t length = 0U;

  if (string_message == NULL || string_message->data.data == NULL) {
    return NULL;
  }

  length = strlen(string_message->data.data);
  payload = (char *)malloc(length + 1U);
  if (payload == NULL) {
    return NULL;
  }

  memcpy(payload, string_message->data.data, length + 1U);
  return payload;
}

static char * amr_mqtt_bridge_serialize_battery_state(const void * message)
{
  const sensor_msgs__msg__BatteryState * battery_state =
    (const sensor_msgs__msg__BatteryState *)message;
  amr_mqtt_bridge_string_builder_t builder = {0};
  double percentage = -1.0;

  if (isfinite(battery_state->percentage)) {
    percentage = battery_state->percentage;
    if (percentage <= 1.0) {
      percentage *= 100.0;
    }
  }

  if (!amr_mqtt_bridge_builder_init(&builder, 512U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{") ||
    !amr_mqtt_bridge_append_header(&builder, &battery_state->header) ||
    !amr_mqtt_bridge_builder_appendf(&builder, ",\"voltage\":%.3f", battery_state->voltage) ||
    !amr_mqtt_bridge_builder_appendf(&builder, ",\"current\":%.3f", battery_state->current) ||
    !amr_mqtt_bridge_builder_appendf(&builder, ",\"percentage\":%.2f", percentage) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder, ",\"power_supply_status\":%u",
      (unsigned int)battery_state->power_supply_status) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder, ",\"power_supply_health\":%u",
      (unsigned int)battery_state->power_supply_health) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder, ",\"power_supply_technology\":%u",
      (unsigned int)battery_state->power_supply_technology) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder, ",\"present\":%s",
      battery_state->present ? "true" : "false") ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static bool amr_mqtt_bridge_serialize_message_raw(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  size_t capacity = 1024U;
  rmw_ret_t rmw_rc = RMW_RET_ERROR;

  if (serialized_message == NULL) {
    return false;
  }

  *serialized_message = rmw_get_zero_initialized_serialized_message();
  if (rmw_serialized_message_init(
      serialized_message,
      capacity,
      &g_amr_mqtt_bridge_runtime.allocator) != RMW_RET_OK)
  {
    return false;
  }

  while (capacity <= (1024U * 1024U)) {
    rmw_rc = rmw_serialize(ros_message, type_support, serialized_message);
    if (rmw_rc == RMW_RET_OK) {
      return true;
    }

    capacity *= 2U;
    if (capacity > (1024U * 1024U)) {
      break;
    }
    if (rmw_serialized_message_resize(serialized_message, capacity) != RMW_RET_OK) {
      break;
    }
  }

  (void)rmw_serialized_message_fini(serialized_message);
  *serialized_message = rmw_get_zero_initialized_serialized_message();
  return false;
}

static bool amr_mqtt_bridge_deserialize_twist_raw(
  const void * payload,
  size_t payload_length,
  geometry_msgs__msg__Twist * twist)
{
  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
  rmw_ret_t rmw_rc = RMW_RET_ERROR;

  if (payload == NULL || payload_length == 0U || twist == NULL) {
    return false;
  }

  if (rmw_serialized_message_init(
      &serialized_message,
      payload_length,
      &g_amr_mqtt_bridge_runtime.allocator) != RMW_RET_OK)
  {
    return false;
  }

  memcpy(serialized_message.buffer, payload, payload_length);
  serialized_message.buffer_length = payload_length;
  rmw_rc = rmw_deserialize(
    &serialized_message,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    twist);
  (void)rmw_serialized_message_fini(&serialized_message);
  return rmw_rc == RMW_RET_OK;
}

static bool amr_mqtt_bridge_extract_twist_from_json(
  const char * payload,
  geometry_msgs__msg__Twist * twist)
{
  const char * begin = payload;
  const char * end = payload + strlen(payload);
  const char * linear_begin = NULL;
  const char * linear_end = NULL;
  const char * angular_begin = NULL;
  const char * angular_end = NULL;

  if (payload == NULL || twist == NULL) {
    return false;
  }

  memset(twist, 0, sizeof(*twist));

  if (amr_mqtt_bridge_extract_json_object_in_range(
      begin, end, "linear", &linear_begin, &linear_end))
  {
    (void)amr_mqtt_bridge_extract_json_double_in_range(
      linear_begin, linear_end, "x", &twist->linear.x);
    (void)amr_mqtt_bridge_extract_json_double_in_range(
      linear_begin, linear_end, "y", &twist->linear.y);
    (void)amr_mqtt_bridge_extract_json_double_in_range(
      linear_begin, linear_end, "z", &twist->linear.z);
  } else {
    (void)amr_mqtt_bridge_extract_json_double_in_range(begin, end, "linear_x", &twist->linear.x);
    (void)amr_mqtt_bridge_extract_json_double_in_range(begin, end, "linear_y", &twist->linear.y);
    (void)amr_mqtt_bridge_extract_json_double_in_range(begin, end, "linear_z", &twist->linear.z);
  }

  if (amr_mqtt_bridge_extract_json_object_in_range(
      begin, end, "angular", &angular_begin, &angular_end))
  {
    (void)amr_mqtt_bridge_extract_json_double_in_range(
      angular_begin, angular_end, "x", &twist->angular.x);
    (void)amr_mqtt_bridge_extract_json_double_in_range(
      angular_begin, angular_end, "y", &twist->angular.y);
    (void)amr_mqtt_bridge_extract_json_double_in_range(
      angular_begin, angular_end, "z", &twist->angular.z);
  } else {
    (void)amr_mqtt_bridge_extract_json_double_in_range(begin, end, "angular_x", &twist->angular.x);
    (void)amr_mqtt_bridge_extract_json_double_in_range(begin, end, "angular_y", &twist->angular.y);
    (void)amr_mqtt_bridge_extract_json_double_in_range(begin, end, "angular_z", &twist->angular.z);
  }

  return true;
}

static bool amr_mqtt_bridge_ensure_directory_exists(const char * path)
{
  char buffer[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH];
  size_t length = 0U;

  if (path == NULL || path[0] == '\0') {
    return false;
  }

  length = strlen(path);
  if (length >= sizeof(buffer)) {
    return false;
  }

  memcpy(buffer, path, length + 1U);

  for (char * cursor = buffer + 1; *cursor != '\0'; ++cursor) {
    if (*cursor != '/') {
      continue;
    }
    *cursor = '\0';
    if (mkdir(buffer, 0775) != 0 && errno != EEXIST) {
      return false;
    }
    *cursor = '/';
  }

  if (mkdir(buffer, 0775) != 0 && errno != EEXIST) {
    return false;
  }

  return true;
}

static bool amr_mqtt_bridge_is_valid_map_basename(const char * basename)
{
  for (size_t index = 0U; basename != NULL && basename[index] != '\0'; ++index) {
    const char ch = basename[index];
    const bool alpha_numeric =
      (ch >= 'a' && ch <= 'z') ||
      (ch >= 'A' && ch <= 'Z') ||
      (ch >= '0' && ch <= '9');
    if (!alpha_numeric && ch != '_' && ch != '-' && ch != '.') {
      return false;
    }
  }

  return basename != NULL && basename[0] != '\0';
}

static bool amr_mqtt_bridge_write_temp_map_files(
  const nav_msgs__msg__OccupancyGrid * map,
  const char * directory,
  const char * basename,
  char * image_path,
  size_t image_path_capacity,
  char * yaml_path,
  size_t yaml_path_capacity)
{
  FILE * image_file = NULL;
  FILE * yaml_file = NULL;

  if (map == NULL || directory == NULL || basename == NULL ||
    map->info.width == 0U || map->info.height == 0U || map->data.size == 0U)
  {
    return false;
  }

  if (!amr_mqtt_bridge_ensure_directory_exists(directory)) {
    return false;
  }

  (void)snprintf(image_path, image_path_capacity, "%s/%s.pgm", directory, basename);
  (void)snprintf(yaml_path, yaml_path_capacity, "%s/%s.yaml", directory, basename);

  image_file = fopen(image_path, "wb");
  if (image_file == NULL) {
    return false;
  }

  (void)fprintf(
    image_file,
    "P5\n%u %u\n255\n",
    (unsigned int)map->info.width,
    (unsigned int)map->info.height);

  for (size_t row = 0U; row < map->info.height; ++row) {
    const size_t map_row = map->info.height - 1U - row;
    for (size_t col = 0U; col < map->info.width; ++col) {
      const size_t index = (map_row * map->info.width) + col;
      const int8_t cell = map->data.data[index];
      uint8_t pixel = 205U;
      if (cell == 0) {
        pixel = 254U;
      } else if (cell >= 50) {
        pixel = 0U;
      }
      (void)fwrite(&pixel, sizeof(pixel), 1U, image_file);
    }
  }
  (void)fclose(image_file);
  image_file = NULL;

  yaml_file = fopen(yaml_path, "wb");
  if (yaml_file == NULL) {
    return false;
  }
  (void)fprintf(yaml_file, "image: %s.pgm\n", basename);
  (void)fprintf(yaml_file, "resolution: %.9f\n", map->info.resolution);
  (void)fprintf(
    yaml_file,
    "origin: [%.9f, %.9f, %.9f]\n",
    map->info.origin.position.x,
    map->info.origin.position.y,
    amr_mqtt_bridge_quaternion_to_yaw(
      map->info.origin.orientation.x,
      map->info.origin.orientation.y,
      map->info.origin.orientation.z,
      map->info.origin.orientation.w));
  (void)fprintf(yaml_file, "negate: 0\n");
  (void)fprintf(yaml_file, "occupied_thresh: 0.65\n");
  (void)fprintf(yaml_file, "free_thresh: 0.196\n");
  (void)fclose(yaml_file);

  return true;
}

static void amr_mqtt_bridge_handle_save_map_command(const char * payload)
{
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  char basename[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  char image_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  char yaml_path[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  char message[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  const nav_msgs__msg__OccupancyGrid * map = &g_amr_mqtt_bridge_ros_state.temp_map_message;

  if (payload == NULL) {
    return;
  }

  (void)amr_mqtt_bridge_extract_json_string_in_range(
    payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id));
  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "basename", basename, sizeof(basename)) ||
    !amr_mqtt_bridge_is_valid_map_basename(basename))
  {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_save_map,
      request_id,
      false,
      "invalid map basename");
    return;
  }

  if (map->info.width == 0U || map->info.height == 0U || map->data.size == 0U) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_save_map,
      request_id,
      false,
      "temporary map is not available");
    return;
  }

  if (!amr_mqtt_bridge_write_temp_map_files(
      map,
      g_amr_mqtt_bridge_config.ros.save_directory,
      basename,
      image_path,
      sizeof(image_path),
      yaml_path,
      sizeof(yaml_path)))
  {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_save_map,
      request_id,
      false,
      "failed to save map files");
    return;
  }

  (void)snprintf(message, sizeof(message), "saved map files");
  amr_mqtt_bridge_publish_simple_response(
    g_amr_mqtt_bridge_config.mqtt.response_save_map,
    request_id,
    true,
    message);
}

static void amr_mqtt_bridge_telemetry_callback(const void * message, void * context)
{
  amr_mqtt_bridge_telemetry_endpoint_t * endpoint =
    (amr_mqtt_bridge_telemetry_endpoint_t *)context;
  char * payload = NULL;
  char viz_topic[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
  const uint64_t now_ms = amr_mqtt_bridge_now_ms();

  if (message == NULL || endpoint == NULL) {
    return;
  }

  if (endpoint->raw_passthrough) {
    const bool should_publish_raw =
      (!endpoint->raw_publish_once || !endpoint->raw_published_once) &&
      (endpoint->raw_min_period_ms == 0U ||
      now_ms >= (endpoint->last_raw_publish_ms + endpoint->raw_min_period_ms));

    if (should_publish_raw) {
      if (!amr_mqtt_bridge_serialize_message_raw(
          message,
          endpoint->type_support,
          &serialized_message))
      {
        return;
      }
      (void)amr_mqtt_bridge_publish_binary_payload(
        endpoint->mqtt_topic,
        serialized_message.buffer,
        serialized_message.buffer_length,
        endpoint->mqtt_qos,
        endpoint->retained);
      (void)rmw_serialized_message_fini(&serialized_message);
      endpoint->last_raw_publish_ms = now_ms;
      endpoint->raw_published_once = true;
    }

    if (endpoint->serializer != NULL &&
      amr_mqtt_bridge_build_viz_topic(endpoint->mqtt_topic, viz_topic, sizeof(viz_topic)))
    {
      const bool should_publish_viz =
        (!endpoint->viz_publish_once || !endpoint->viz_published_once) &&
        (endpoint->viz_min_period_ms == 0U ||
        now_ms >= (endpoint->last_viz_publish_ms + endpoint->viz_min_period_ms));
      if (should_publish_viz) {
        payload = endpoint->serializer(message);
        if (payload != NULL) {
          (void)amr_mqtt_bridge_publish_payload(
            viz_topic,
            payload,
            endpoint->mqtt_qos,
            endpoint->retained);
          free(payload);
          endpoint->last_viz_publish_ms = now_ms;
          endpoint->viz_published_once = true;
        }
      }
    }
    return;
  }

  if (endpoint->serializer == NULL) {
    return;
  }

  payload = endpoint->serializer(message);
  if (payload == NULL) {
    return;
  }
  (void)amr_mqtt_bridge_publish_payload(
    endpoint->mqtt_topic,
    payload,
    endpoint->mqtt_qos,
    endpoint->retained);
  free(payload);
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
  int mqtt_qos,
  bool retained,
  bool raw_passthrough,
  amr_mqtt_bridge_serializer_fn_t serializer,
  bool raw_publish_once,
  bool viz_publish_once,
  uint64_t raw_min_period_ms,
  uint64_t viz_min_period_ms)
{
  memset(endpoint, 0, sizeof(*endpoint));
  endpoint->label = label;
  endpoint->ros_topic = ros_topic;
  endpoint->mqtt_topic = mqtt_topic;
  endpoint->subscription = subscription;
  endpoint->message = message;
  endpoint->type_support = type_support;
  endpoint->qos_profile = qos_profile;
  endpoint->mqtt_qos = mqtt_qos;
  endpoint->retained = retained;
  endpoint->raw_passthrough = raw_passthrough;
  endpoint->serializer = serializer;
  endpoint->raw_publish_once = raw_publish_once;
  endpoint->viz_publish_once = viz_publish_once;
  endpoint->raw_min_period_ms = raw_min_period_ms;
  endpoint->viz_min_period_ms = viz_min_period_ms;
}

static int amr_mqtt_bridge_add_subscription(
  amr_mqtt_bridge_telemetry_endpoint_t * endpoint)
{
  rcl_ret_t rc = rclc_subscription_init(
    endpoint->subscription,
    &g_amr_mqtt_bridge_runtime.node,
    endpoint->type_support,
    endpoint->ros_topic,
    endpoint->qos_profile);
  if (rc != RCL_RET_OK) {
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
    rcl_reset_error();
    return 1;
  }

  return 0;
}

static int amr_mqtt_bridge_init_ros_interfaces(void)
{
  rcl_ret_t rc;

  memset(&g_amr_mqtt_bridge_ros_state, 0, sizeof(g_amr_mqtt_bridge_ros_state));
  if (!nav_msgs__msg__OccupancyGrid__init(&g_amr_mqtt_bridge_ros_state.map_message) ||
    !geometry_msgs__msg__PoseStamped__init(&g_amr_mqtt_bridge_ros_state.robot_pose_message) ||
    !nav_msgs__msg__Path__init(&g_amr_mqtt_bridge_ros_state.global_path_message) ||
    !nav_msgs__msg__Path__init(&g_amr_mqtt_bridge_ros_state.local_path_message) ||
    !nav_msgs__msg__OccupancyGrid__init(&g_amr_mqtt_bridge_ros_state.global_costmap_message) ||
    !nav_msgs__msg__OccupancyGrid__init(&g_amr_mqtt_bridge_ros_state.local_costmap_message) ||
    !amr_msgs__msg__MotionStatus__init(&g_amr_mqtt_bridge_ros_state.motion_status_message) ||
    !sensor_msgs__msg__LaserScan__init(&g_amr_mqtt_bridge_ros_state.scan_message) ||
    !nav_msgs__msg__Odometry__init(&g_amr_mqtt_bridge_ros_state.odom_message) ||
    !sensor_msgs__msg__Imu__init(&g_amr_mqtt_bridge_ros_state.imu_message) ||
    !tf2_msgs__msg__TFMessage__init(&g_amr_mqtt_bridge_ros_state.tf_message) ||
    !tf2_msgs__msg__TFMessage__init(&g_amr_mqtt_bridge_ros_state.tf_static_message) ||
    !sensor_msgs__msg__JointState__init(&g_amr_mqtt_bridge_ros_state.joint_states_message) ||
    !std_msgs__msg__String__init(&g_amr_mqtt_bridge_ros_state.robot_description_message) ||
    !sensor_msgs__msg__BatteryState__init(&g_amr_mqtt_bridge_ros_state.battery_state_message) ||
    !nav_msgs__msg__OccupancyGrid__init(&g_amr_mqtt_bridge_ros_state.temp_map_message) ||
    !geometry_msgs__msg__PoseStamped__init(&g_amr_mqtt_bridge_ros_state.mapping_pose_message) ||
    !std_msgs__msg__String__init(&g_amr_mqtt_bridge_ros_state.slam_graph_message) ||
    !geometry_msgs__msg__Twist__init(&g_amr_mqtt_bridge_ros_state.cmd_vel_message) ||
    !action_msgs__msg__GoalStatusArray__init(&g_amr_mqtt_bridge_ros_state.navigate_status_message))
  {
    return 1;
  }
  g_amr_mqtt_bridge_ros_state.messages_initialized = true;

  g_amr_mqtt_bridge_ros_state.map_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.robot_pose_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.global_path_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.local_path_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.global_costmap_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.local_costmap_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.motion_status_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.scan_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.odom_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.imu_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.tf_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.tf_static_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.joint_states_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.robot_description_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.battery_state_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.temp_map_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.mapping_pose_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.slam_graph_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_bridge_ros_state.telemetry_endpoint_count = AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS;

  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[0],
    "map",
    g_amr_mqtt_bridge_config.ros.topic_map,
    g_amr_mqtt_bridge_config.mqtt.telemetry_map,
    &g_amr_mqtt_bridge_ros_state.map_subscription,
    &g_amr_mqtt_bridge_ros_state.map_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, OccupancyGrid)(),
    &k_transient_local_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    true,
    true,
    amr_mqtt_bridge_serialize_occupancy_grid,
    true,
    true,
    0U,
    0U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[1],
    "robot_pose",
    g_amr_mqtt_bridge_config.ros.topic_robot_pose,
    g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose,
    &g_amr_mqtt_bridge_ros_state.robot_pose_subscription,
    &g_amr_mqtt_bridge_ros_state.robot_pose_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, geometry_msgs, msg, PoseStamped)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_pose_stamped,
    false,
    false,
    0U,
    50U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[2],
    "global_path",
    g_amr_mqtt_bridge_config.ros.topic_global_path,
    g_amr_mqtt_bridge_config.mqtt.telemetry_global_path,
    &g_amr_mqtt_bridge_ros_state.global_path_subscription,
    &g_amr_mqtt_bridge_ros_state.global_path_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, Path)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_path,
    false,
    false,
    0U,
    200U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[3],
    "local_path",
    g_amr_mqtt_bridge_config.ros.topic_local_path,
    g_amr_mqtt_bridge_config.mqtt.telemetry_local_path,
    &g_amr_mqtt_bridge_ros_state.local_path_subscription,
    &g_amr_mqtt_bridge_ros_state.local_path_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, Path)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_path,
    false,
    false,
    0U,
    100U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[4],
    "global_costmap",
    g_amr_mqtt_bridge_config.ros.topic_global_costmap,
    g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap,
    &g_amr_mqtt_bridge_ros_state.global_costmap_subscription,
    &g_amr_mqtt_bridge_ros_state.global_costmap_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, OccupancyGrid)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_occupancy_grid,
    false,
    false,
    0U,
    300U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[5],
    "local_costmap",
    g_amr_mqtt_bridge_config.ros.topic_local_costmap,
    g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap,
    &g_amr_mqtt_bridge_ros_state.local_costmap_subscription,
    &g_amr_mqtt_bridge_ros_state.local_costmap_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, OccupancyGrid)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_occupancy_grid,
    false,
    false,
    0U,
    150U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[6],
    "motion_status",
    g_amr_mqtt_bridge_config.ros.topic_motion_status,
    g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status,
    &g_amr_mqtt_bridge_ros_state.motion_status_subscription,
    &g_amr_mqtt_bridge_ros_state.motion_status_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, amr_msgs, msg, MotionStatus)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_motion_status,
    false,
    false,
    0U,
    100U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[7],
    "scan",
    g_amr_mqtt_bridge_config.ros.topic_scan,
    g_amr_mqtt_bridge_config.mqtt.telemetry_scan,
    &g_amr_mqtt_bridge_ros_state.scan_subscription,
    &g_amr_mqtt_bridge_ros_state.scan_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, sensor_msgs, msg, LaserScan)(),
    &k_sensor_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_scan,
    false,
    false,
    0U,
    100U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[8],
    "odom",
    g_amr_mqtt_bridge_config.ros.topic_odom,
    g_amr_mqtt_bridge_config.mqtt.telemetry_odom,
    &g_amr_mqtt_bridge_ros_state.odom_subscription,
    &g_amr_mqtt_bridge_ros_state.odom_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, Odometry)(),
    &k_sensor_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    NULL,
    false,
    false,
    0U,
    0U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[9],
    "imu",
    g_amr_mqtt_bridge_config.ros.topic_imu,
    g_amr_mqtt_bridge_config.mqtt.telemetry_imu,
    &g_amr_mqtt_bridge_ros_state.imu_subscription,
    &g_amr_mqtt_bridge_ros_state.imu_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, sensor_msgs, msg, Imu)(),
    &k_sensor_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    NULL,
    false,
    false,
    0U,
    0U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[10],
    "tf",
    g_amr_mqtt_bridge_config.ros.topic_tf,
    g_amr_mqtt_bridge_config.mqtt.telemetry_tf,
    &g_amr_mqtt_bridge_ros_state.tf_subscription,
    &g_amr_mqtt_bridge_ros_state.tf_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, tf2_msgs, msg, TFMessage)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_tf_message,
    false,
    false,
    0U,
    50U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[11],
    "tf_static",
    g_amr_mqtt_bridge_config.ros.topic_tf_static,
    g_amr_mqtt_bridge_config.mqtt.telemetry_tf_static,
    &g_amr_mqtt_bridge_ros_state.tf_static_subscription,
    &g_amr_mqtt_bridge_ros_state.tf_static_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, tf2_msgs, msg, TFMessage)(),
    &k_transient_local_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    true,
    true,
    amr_mqtt_bridge_serialize_tf_message,
    true,
    true,
    0U,
    0U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[12],
    "joint_states",
    g_amr_mqtt_bridge_config.ros.topic_joint_states,
    g_amr_mqtt_bridge_config.mqtt.telemetry_joint_states,
    &g_amr_mqtt_bridge_ros_state.joint_states_subscription,
    &g_amr_mqtt_bridge_ros_state.joint_states_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, sensor_msgs, msg, JointState)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    NULL,
    false,
    false,
    0U,
    0U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[13],
    "robot_description",
    g_amr_mqtt_bridge_config.ros.topic_robot_description,
    g_amr_mqtt_bridge_config.mqtt.telemetry_robot_description,
    &g_amr_mqtt_bridge_ros_state.robot_description_subscription,
    &g_amr_mqtt_bridge_ros_state.robot_description_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, std_msgs, msg, String)(),
    &k_transient_local_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    true,
    true,
    amr_mqtt_bridge_serialize_string_message,
    true,
    true,
    0U,
    0U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[14],
    "battery_state",
    g_amr_mqtt_bridge_config.ros.topic_battery_state,
    g_amr_mqtt_bridge_config.mqtt.telemetry_battery_state,
    &g_amr_mqtt_bridge_ros_state.battery_state_subscription,
    &g_amr_mqtt_bridge_ros_state.battery_state_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, sensor_msgs, msg, BatteryState)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_battery_state,
    false,
    false,
    0U,
    1000U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[15],
    "temp_map",
    g_amr_mqtt_bridge_config.ros.topic_temp_map,
    g_amr_mqtt_bridge_config.mqtt.telemetry_temp_map,
    &g_amr_mqtt_bridge_ros_state.temp_map_subscription,
    &g_amr_mqtt_bridge_ros_state.temp_map_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, OccupancyGrid)(),
    &k_transient_local_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_occupancy_grid,
    false,
    false,
    0U,
    300U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[16],
    "mapping_pose",
    g_amr_mqtt_bridge_config.ros.topic_mapping_pose,
    g_amr_mqtt_bridge_config.mqtt.telemetry_mapping_pose,
    &g_amr_mqtt_bridge_ros_state.mapping_pose_subscription,
    &g_amr_mqtt_bridge_ros_state.mapping_pose_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, geometry_msgs, msg, PoseStamped)(),
    &k_default_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_pose_stamped,
    false,
    false,
    0U,
    100U);
  amr_mqtt_bridge_configure_endpoint(
    &g_amr_mqtt_bridge_ros_state.telemetry_endpoints[17],
    "slam_graph",
    g_amr_mqtt_bridge_config.ros.topic_slam_graph,
    g_amr_mqtt_bridge_config.mqtt.telemetry_slam_graph,
    &g_amr_mqtt_bridge_ros_state.slam_graph_subscription,
    &g_amr_mqtt_bridge_ros_state.slam_graph_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, std_msgs, msg, String)(),
    &k_transient_local_qos,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
    false,
    true,
    amr_mqtt_bridge_serialize_string_json_message,
    false,
    false,
    0U,
    250U);

  for (size_t index = 0; index < g_amr_mqtt_bridge_ros_state.telemetry_endpoint_count; ++index) {
    if (amr_mqtt_bridge_add_subscription(&g_amr_mqtt_bridge_ros_state.telemetry_endpoints[index]) != 0) {
      return 1;
    }
  }
  g_amr_mqtt_bridge_ros_state.subscriptions_initialized = true;

  g_amr_mqtt_bridge_ros_state.cmd_vel_publisher = rcl_get_zero_initialized_publisher();
  g_amr_mqtt_bridge_ros_state.initial_pose_publisher = rcl_get_zero_initialized_publisher();
  g_amr_mqtt_bridge_ros_state.plan_segment_client = rcl_get_zero_initialized_client();
  g_amr_mqtt_bridge_ros_state.plan_route_client = rcl_get_zero_initialized_client();
  g_amr_mqtt_bridge_ros_state.navigate_to_pose_client = rcl_action_get_zero_initialized_client();
  g_amr_mqtt_bridge_ros_state.navigate_wait_set = rcl_get_zero_initialized_wait_set();
  rc = rclc_publisher_init_default(
    &g_amr_mqtt_bridge_ros_state.cmd_vel_publisher,
    &g_amr_mqtt_bridge_runtime.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    g_amr_mqtt_bridge_config.ros.topic_cmd_vel);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }
  g_amr_mqtt_bridge_ros_state.cmd_vel_publisher_initialized = true;

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

  {
    size_t num_subscriptions = 0U;
    size_t num_guard_conditions = 0U;
    size_t num_timers = 0U;
    size_t num_clients = 0U;
    size_t num_services = 0U;

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
  }
  g_amr_mqtt_bridge_ros_state.navigate_wait_set_initialized = true;
  g_amr_mqtt_bridge_ros_state.navigate_status_message_initialized = true;
  memset(&g_amr_mqtt_bridge_ros_state.navigate_state, 0, sizeof(g_amr_mqtt_bridge_ros_state.navigate_state));
  return 0;
}

static void amr_mqtt_bridge_fini_ros_interfaces(void)
{
  if (g_amr_mqtt_bridge_ros_state.subscriptions_initialized) {
    for (size_t index = 0; index < g_amr_mqtt_bridge_ros_state.telemetry_endpoint_count; ++index) {
      if (g_amr_mqtt_bridge_ros_state.telemetry_endpoints[index].subscription != NULL) {
        amr_mqtt_bridge_log_rcl_error(
          "subscription",
          rcl_subscription_fini(
            g_amr_mqtt_bridge_ros_state.telemetry_endpoints[index].subscription,
            &g_amr_mqtt_bridge_runtime.node));
      }
    }
    g_amr_mqtt_bridge_ros_state.subscriptions_initialized = false;
  }

  if (g_amr_mqtt_bridge_ros_state.cmd_vel_publisher_initialized) {
    amr_mqtt_bridge_log_rcl_error(
      "cmd_vel publisher",
      rcl_publisher_fini(
        &g_amr_mqtt_bridge_ros_state.cmd_vel_publisher,
        &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.cmd_vel_publisher_initialized = false;
  }

  if (g_amr_mqtt_bridge_ros_state.navigate_wait_set_initialized) {
    amr_mqtt_bridge_log_rcl_error(
      "navigate wait set",
      rcl_wait_set_fini(&g_amr_mqtt_bridge_ros_state.navigate_wait_set));
    g_amr_mqtt_bridge_ros_state.navigate_wait_set_initialized = false;
  }
  if (g_amr_mqtt_bridge_ros_state.navigate_to_pose_client_initialized) {
    amr_mqtt_bridge_log_rcl_error(
      "navigate action client",
      rcl_action_client_fini(
        &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
        &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.navigate_to_pose_client_initialized = false;
  }
  if (g_amr_mqtt_bridge_ros_state.plan_route_client_initialized) {
    amr_mqtt_bridge_log_rcl_error(
      "plan_route client",
      rcl_client_fini(&g_amr_mqtt_bridge_ros_state.plan_route_client, &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.plan_route_client_initialized = false;
  }
  if (g_amr_mqtt_bridge_ros_state.plan_segment_client_initialized) {
    amr_mqtt_bridge_log_rcl_error(
      "plan_segment client",
      rcl_client_fini(&g_amr_mqtt_bridge_ros_state.plan_segment_client, &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.plan_segment_client_initialized = false;
  }
  if (g_amr_mqtt_bridge_ros_state.initial_pose_publisher_initialized) {
    amr_mqtt_bridge_log_rcl_error(
      "initial_pose publisher",
      rcl_publisher_fini(
        &g_amr_mqtt_bridge_ros_state.initial_pose_publisher,
        &g_amr_mqtt_bridge_runtime.node));
    g_amr_mqtt_bridge_ros_state.initial_pose_publisher_initialized = false;
  }

  if (g_amr_mqtt_bridge_ros_state.messages_initialized) {
    geometry_msgs__msg__PoseStamped__fini(&g_amr_mqtt_bridge_ros_state.robot_pose_message);
    nav_msgs__msg__Path__fini(&g_amr_mqtt_bridge_ros_state.global_path_message);
    nav_msgs__msg__Path__fini(&g_amr_mqtt_bridge_ros_state.local_path_message);
    nav_msgs__msg__OccupancyGrid__fini(&g_amr_mqtt_bridge_ros_state.global_costmap_message);
    nav_msgs__msg__OccupancyGrid__fini(&g_amr_mqtt_bridge_ros_state.local_costmap_message);
    amr_msgs__msg__MotionStatus__fini(&g_amr_mqtt_bridge_ros_state.motion_status_message);
    nav_msgs__msg__OccupancyGrid__fini(&g_amr_mqtt_bridge_ros_state.map_message);
    sensor_msgs__msg__LaserScan__fini(&g_amr_mqtt_bridge_ros_state.scan_message);
    nav_msgs__msg__Odometry__fini(&g_amr_mqtt_bridge_ros_state.odom_message);
    sensor_msgs__msg__Imu__fini(&g_amr_mqtt_bridge_ros_state.imu_message);
    tf2_msgs__msg__TFMessage__fini(&g_amr_mqtt_bridge_ros_state.tf_message);
    tf2_msgs__msg__TFMessage__fini(&g_amr_mqtt_bridge_ros_state.tf_static_message);
    sensor_msgs__msg__JointState__fini(&g_amr_mqtt_bridge_ros_state.joint_states_message);
    std_msgs__msg__String__fini(&g_amr_mqtt_bridge_ros_state.robot_description_message);
    sensor_msgs__msg__BatteryState__fini(&g_amr_mqtt_bridge_ros_state.battery_state_message);
    nav_msgs__msg__OccupancyGrid__fini(&g_amr_mqtt_bridge_ros_state.temp_map_message);
    geometry_msgs__msg__PoseStamped__fini(&g_amr_mqtt_bridge_ros_state.mapping_pose_message);
    std_msgs__msg__String__fini(&g_amr_mqtt_bridge_ros_state.slam_graph_message);
    geometry_msgs__msg__Twist__fini(&g_amr_mqtt_bridge_ros_state.cmd_vel_message);
    action_msgs__msg__GoalStatusArray__fini(&g_amr_mqtt_bridge_ros_state.navigate_status_message);
    g_amr_mqtt_bridge_ros_state.messages_initialized = false;
  }
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

static bool amr_mqtt_bridge_extract_json_bool_in_range(
  const char * begin,
  const char * end,
  const char * key,
  bool * output)
{
  const char * value = amr_mqtt_bridge_find_value_for_key(begin, end, key);

  if (value == NULL || output == NULL) {
    return false;
  }

  if ((size_t)(end - value) >= 4U && strncmp(value, "true", 4U) == 0) {
    *output = true;
    return true;
  }
  if ((size_t)(end - value) >= 5U && strncmp(value, "false", 5U) == 0) {
    *output = false;
    return true;
  }

  return false;
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

static void amr_mqtt_bridge_fill_quaternion_from_yaw(
  double yaw,
  geometry_msgs__msg__Quaternion * orientation)
{
  if (orientation == NULL) {
    return;
  }

  orientation->x = 0.0;
  orientation->y = 0.0;
  orientation->z = sin(yaw * 0.5);
  orientation->w = cos(yaw * 0.5);
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
  (void)rosidl_runtime_c__String__assign(&pose->header.frame_id, frame_id);
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

static char * amr_mqtt_bridge_serialize_ping_response(
  const char * request_id,
  bool success,
  double sent_at_ms,
  uint64_t bridge_time_ms,
  const char * message)
{
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 320U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"request_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, request_id) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"success\":%s,\"sent_at_ms\":%.3f,\"bridge_time_ms\":%llu,\"message\":",
      success ? "true" : "false",
      sent_at_ms,
      (unsigned long long)bridge_time_ms) ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, message) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
}

static char * amr_mqtt_bridge_serialize_navigate_response(
  const char * request_id,
  bool success,
  int status_code,
  bool accepted,
  bool completed,
  const char * message)
{
  amr_mqtt_bridge_string_builder_t builder = {0};
  if (!amr_mqtt_bridge_builder_init(&builder, 320U) ||
    !amr_mqtt_bridge_builder_append(&builder, "{\"request_id\":") ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, request_id) ||
    !amr_mqtt_bridge_builder_appendf(
      &builder,
      ",\"success\":%s,\"accepted\":%s,\"completed\":%s,\"status_code\":%d,\"message\":",
      success ? "true" : "false",
      accepted ? "true" : "false",
      completed ? "true" : "false",
      status_code) ||
    !amr_mqtt_bridge_builder_append_json_string(&builder, message) ||
    !amr_mqtt_bridge_builder_append(&builder, "}"))
  {
    amr_mqtt_bridge_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_bridge_builder_take(&builder);
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
    g_amr_mqtt_bridge_config.mqtt.service_qos,
    false);
  free(payload);
}

static void amr_mqtt_bridge_publish_navigate_response(
  const char * request_id,
  bool success,
  int status_code,
  bool accepted,
  bool completed,
  const char * message)
{
  char * payload = amr_mqtt_bridge_serialize_navigate_response(
    request_id,
    success,
    status_code,
    accepted,
    completed,
    message);
  if (payload == NULL) {
    return;
  }
  (void)amr_mqtt_bridge_publish_payload(
    g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
    payload,
    g_amr_mqtt_bridge_config.mqtt.service_qos,
    false);
  free(payload);
}

static void amr_mqtt_bridge_publish_ping_response(
  const char * request_id,
  bool success,
  double sent_at_ms,
  const char * message)
{
  char * payload = amr_mqtt_bridge_serialize_ping_response(
    request_id,
    success,
    sent_at_ms,
    amr_mqtt_bridge_now_ms(),
    message);
  if (payload == NULL) {
    return;
  }
  (void)amr_mqtt_bridge_publish_payload(
    g_amr_mqtt_bridge_config.mqtt.response_ping,
    payload,
    g_amr_mqtt_bridge_config.mqtt.service_qos,
    false);
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
    {
      struct timespec sleep_time = {.tv_sec = 0, .tv_nsec = 100 * 1000 * 1000};
      nanosleep(&sleep_time, NULL);
    }
  }
  return false;
}

static void amr_mqtt_bridge_handle_cmd_vel_message(
  const void * payload,
  size_t payload_length)
{
  rcl_ret_t rc;
  char text_payload[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};

  if (payload != NULL && payload_length > 0U && ((const unsigned char *)payload)[0] == '{') {
    const size_t copy_length =
      payload_length < (sizeof(text_payload) - 1U) ? payload_length : (sizeof(text_payload) - 1U);
    memcpy(text_payload, payload, copy_length);
    text_payload[copy_length] = '\0';
    if (!amr_mqtt_bridge_extract_twist_from_json(
        text_payload,
        &g_amr_mqtt_bridge_ros_state.cmd_vel_message))
    {
      return;
    }
  } else if (!amr_mqtt_bridge_deserialize_twist_raw(
      payload,
      payload_length,
      &g_amr_mqtt_bridge_ros_state.cmd_vel_message))
  {
    return;
  }

  rc = rcl_publish(
    &g_amr_mqtt_bridge_ros_state.cmd_vel_publisher,
    &g_amr_mqtt_bridge_ros_state.cmd_vel_message,
    NULL);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
  }
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
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  const char * start_begin = NULL;
  const char * start_end = NULL;
  const char * goal_begin = NULL;
  const char * goal_end = NULL;
  int64_t sequence_number = 0;
  rcl_ret_t rc;

  memset(&request, 0, sizeof(request));
  if (!amr_msgs__srv__PlanSegment_Request__init(&request)) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
      "",
      false,
      "failed to allocate request");
    return;
  }

  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id)) ||
    !amr_mqtt_bridge_extract_json_object_in_range(payload, payload + strlen(payload), "start", &start_begin, &start_end) ||
    !amr_mqtt_bridge_extract_json_object_in_range(payload, payload + strlen(payload), "goal", &goal_begin, &goal_end) ||
    !amr_mqtt_bridge_parse_pose_stamped_in_range(start_begin, start_end, &request.start) ||
    !amr_mqtt_bridge_parse_pose_stamped_in_range(goal_begin, goal_end, &request.goal))
  {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
      request_id[0] != '\0' ? request_id : "",
      false,
      "invalid plan_segment payload");
    amr_msgs__srv__PlanSegment_Request__fini(&request);
    return;
  }

  if (!amr_mqtt_bridge_wait_for_service_available(
      &g_amr_mqtt_bridge_ros_state.plan_segment_client, 1000))
  {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
      request_id,
      false,
      "plan_segment service unavailable");
    amr_msgs__srv__PlanSegment_Request__fini(&request);
    return;
  }

  rc = rcl_send_request(
    &g_amr_mqtt_bridge_ros_state.plan_segment_client,
    &request,
    &sequence_number);
  amr_msgs__srv__PlanSegment_Request__fini(&request);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
      request_id,
      false,
      "failed to dispatch plan_segment request");
    return;
  }

  amr_mqtt_bridge_publish_simple_response(
    g_amr_mqtt_bridge_config.mqtt.response_plan_segment,
    request_id,
    true,
    "plan_segment request dispatched");
}

static void amr_mqtt_bridge_handle_plan_route_request(const char * payload)
{
  amr_msgs__srv__PlanRoute_Request request;
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  const char * start_begin = NULL;
  const char * start_end = NULL;
  const char * waypoints_begin = NULL;
  const char * waypoints_end = NULL;
  int64_t sequence_number = 0;
  rcl_ret_t rc;

  memset(&request, 0, sizeof(request));
  if (!amr_msgs__srv__PlanRoute_Request__init(&request)) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_route,
      "",
      false,
      "failed to allocate request");
    return;
  }

  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id)) ||
    !amr_mqtt_bridge_extract_json_object_in_range(payload, payload + strlen(payload), "start", &start_begin, &start_end) ||
    !amr_mqtt_bridge_extract_json_array_in_range(payload, payload + strlen(payload), "waypoints", &waypoints_begin, &waypoints_end) ||
    !amr_mqtt_bridge_parse_pose_stamped_in_range(start_begin, start_end, &request.start) ||
    !amr_mqtt_bridge_parse_waypoints_array(waypoints_begin, waypoints_end, &request.waypoints))
  {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_route,
      request_id[0] != '\0' ? request_id : "",
      false,
      "invalid plan_route payload");
    amr_msgs__srv__PlanRoute_Request__fini(&request);
    return;
  }

  if (!amr_mqtt_bridge_wait_for_service_available(
      &g_amr_mqtt_bridge_ros_state.plan_route_client, 1000))
  {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_route,
      request_id,
      false,
      "plan_route service unavailable");
    amr_msgs__srv__PlanRoute_Request__fini(&request);
    return;
  }

  rc = rcl_send_request(
    &g_amr_mqtt_bridge_ros_state.plan_route_client,
    &request,
    &sequence_number);
  amr_msgs__srv__PlanRoute_Request__fini(&request);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_plan_route,
      request_id,
      false,
      "failed to dispatch plan_route request");
    return;
  }

  amr_mqtt_bridge_publish_simple_response(
    g_amr_mqtt_bridge_config.mqtt.response_plan_route,
    request_id,
    true,
    "plan_route request dispatched");
}

static void amr_mqtt_bridge_handle_navigate_to_pose_command(const char * payload)
{
  amr_msgs__action__NavigateToPose_SendGoal_Request request;
  const char * goal_begin = NULL;
  const char * goal_end = NULL;
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  bool cancel_requested = false;
  bool is_available = false;
  int64_t sequence_number = 0;
  rcl_ret_t rc;

  memset(&request, 0, sizeof(request));
  if (!amr_msgs__action__NavigateToPose_SendGoal_Request__init(&request)) {
    return;
  }

  (void)amr_mqtt_bridge_extract_json_string_in_range(
    payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id));
  if (amr_mqtt_bridge_extract_json_bool_in_range(
      payload, payload + strlen(payload), "cancel", &cancel_requested) && cancel_requested)
  {
    action_msgs__srv__CancelGoal_Request cancel_request;

    if (!g_amr_mqtt_bridge_ros_state.navigate_state.active) {
      amr_mqtt_bridge_publish_simple_response(
        g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
        request_id,
        false,
        "no active navigate_to_pose goal");
      amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
      return;
    }

    memset(&cancel_request, 0, sizeof(cancel_request));
    if (!action_msgs__srv__CancelGoal_Request__init(&cancel_request)) {
      amr_mqtt_bridge_publish_simple_response(
        g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
        request_id,
        false,
        "failed to initialize cancel request");
      amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
      return;
    }

    memcpy(
      cancel_request.goal_info.goal_id.uuid,
      g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid,
      sizeof(g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid));
    cancel_request.goal_info.stamp.sec = 0;
    cancel_request.goal_info.stamp.nanosec = 0U;

    rc = rcl_action_send_cancel_request(
      &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
      &cancel_request,
      &sequence_number);
    action_msgs__srv__CancelGoal_Request__fini(&cancel_request);
    amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
    if (rc != RCL_RET_OK) {
      rcl_reset_error();
      amr_mqtt_bridge_publish_simple_response(
        g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
        request_id,
        false,
        "failed to dispatch goal cancel");
      return;
    }

    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
      request_id,
      true,
      "goal cancel dispatched");
    return;
  }

  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id)) ||
    !amr_mqtt_bridge_extract_json_object_in_range(payload, payload + strlen(payload), "goal_pose", &goal_begin, &goal_end) ||
    !amr_mqtt_bridge_parse_pose_stamped_in_range(goal_begin, goal_end, &request.goal.goal_pose))
  {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
      request_id[0] != '\0' ? request_id : "",
      false,
      "invalid navigate_to_pose payload");
    amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
    return;
  }

  if (g_amr_mqtt_bridge_ros_state.navigate_state.active) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
      request_id,
      false,
      "navigate_to_pose goal already active");
    amr_msgs__action__NavigateToPose_SendGoal_Request__fini(&request);
    return;
  }

  rc = rcl_action_server_is_available(
    &g_amr_mqtt_bridge_runtime.node,
    &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
    &is_available);
  if (rc != RCL_RET_OK || !is_available) {
    rcl_reset_error();
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
      request_id,
      false,
      "navigate_to_pose action unavailable");
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
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
      request_id,
      false,
      "failed to send navigate_to_pose goal");
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

static void amr_mqtt_bridge_handle_navigate_cancel_command(const char * payload)
{
  action_msgs__srv__CancelGoal_Request cancel_request;
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  int64_t sequence_number = 0;
  rcl_ret_t rc;

  (void)amr_mqtt_bridge_extract_json_string_in_range(
    payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id));

  if (!g_amr_mqtt_bridge_ros_state.navigate_state.active) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
      request_id,
      false,
      "no active navigate_to_pose goal");
    return;
  }

  memset(&cancel_request, 0, sizeof(cancel_request));
  if (!action_msgs__srv__CancelGoal_Request__init(&cancel_request)) {
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
      request_id,
      false,
      "failed to initialize cancel request");
    return;
  }

  memcpy(
    cancel_request.goal_info.goal_id.uuid,
    g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid,
    sizeof(g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid));
  cancel_request.goal_info.stamp.sec = 0;
  cancel_request.goal_info.stamp.nanosec = 0U;

  rc = rcl_action_send_cancel_request(
    &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
    &cancel_request,
    &sequence_number);
  action_msgs__srv__CancelGoal_Request__fini(&cancel_request);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    amr_mqtt_bridge_publish_simple_response(
      g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
      request_id,
      false,
      "failed to dispatch goal cancel");
    return;
  }

  amr_mqtt_bridge_publish_simple_response(
    g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
    request_id,
    true,
    "goal cancel dispatched");
}

static void amr_mqtt_bridge_handle_ping_command(const char * payload)
{
  char request_id[AMR_MQTT_BRIDGE_MAX_STRING_LENGTH] = {0};
  double sent_at_ms = 0.0;

  if (!amr_mqtt_bridge_extract_json_string_in_range(
      payload, payload + strlen(payload), "request_id", request_id, sizeof(request_id)))
  {
    return;
  }

  (void)amr_mqtt_bridge_extract_json_double_in_range(
    payload, payload + strlen(payload), "sent_at_ms", &sent_at_ms);

  amr_mqtt_bridge_publish_ping_response(
    request_id,
    true,
    sent_at_ms,
    "pong");
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

    memset(&response_header, 0, sizeof(response_header));
    memset(&goal_response, 0, sizeof(goal_response));
    rc = rcl_action_take_goal_response(
      &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
      &response_header,
      &goal_response);
    if (rc == RCL_RET_OK &&
      response_header.sequence_number == g_amr_mqtt_bridge_ros_state.navigate_state.goal_request_sequence_number)
    {
      amr_mqtt_bridge_publish_simple_response(
        g_amr_mqtt_bridge_config.mqtt.response_navigate_to_pose,
        g_amr_mqtt_bridge_ros_state.navigate_state.request_id,
        goal_response.accepted,
        goal_response.accepted ?
        "goal accepted; planning and execution pending" :
        "goal rejected");

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
    rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();

    memset(&feedback_message, 0, sizeof(feedback_message));
    if (amr_msgs__action__NavigateToPose_FeedbackMessage__init(&feedback_message)) {
      rc = rcl_action_take_feedback(
        &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
        &feedback_message);
      if (rc == RCL_RET_OK &&
        amr_mqtt_bridge_uuid_equals(
          feedback_message.goal_id.uuid,
          g_amr_mqtt_bridge_ros_state.navigate_state.goal_uuid) &&
        amr_mqtt_bridge_serialize_message_raw(
          &feedback_message,
          ROSIDL_GET_MSG_TYPE_SUPPORT(amr_msgs, action, NavigateToPose_FeedbackMessage),
          &serialized_message))
      {
        (void)amr_mqtt_bridge_publish_binary_payload(
          g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose,
          serialized_message.buffer,
          serialized_message.buffer_length,
          g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
          false);
        (void)rmw_serialized_message_fini(&serialized_message);
      }
      amr_msgs__action__NavigateToPose_FeedbackMessage__fini(&feedback_message);
    }
  }

  if (is_status_ready) {
    rc = rcl_action_take_status(
      &g_amr_mqtt_bridge_ros_state.navigate_to_pose_client,
      &g_amr_mqtt_bridge_ros_state.navigate_status_message);
    if (rc == RCL_RET_OK) {
      rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
      if (amr_mqtt_bridge_serialize_message_raw(
          &g_amr_mqtt_bridge_ros_state.navigate_status_message,
          ROSIDL_GET_MSG_TYPE_SUPPORT(action_msgs, msg, GoalStatusArray),
          &serialized_message))
      {
        (void)amr_mqtt_bridge_publish_binary_payload(
          g_amr_mqtt_bridge_config.mqtt.status_navigate_to_pose,
          serialized_message.buffer,
          serialized_message.buffer_length,
          g_amr_mqtt_bridge_config.mqtt.telemetry_qos,
          false);
        (void)rmw_serialized_message_fini(&serialized_message);
      }
      action_msgs__msg__GoalStatusArray__fini(&g_amr_mqtt_bridge_ros_state.navigate_status_message);
      g_amr_mqtt_bridge_ros_state.navigate_status_message_initialized =
        action_msgs__msg__GoalStatusArray__init(&g_amr_mqtt_bridge_ros_state.navigate_status_message);
    }
  }

  if (is_result_response_ready && g_amr_mqtt_bridge_ros_state.navigate_state.result_response_pending) {
    rmw_request_id_t response_header;
    amr_msgs__action__NavigateToPose_GetResult_Response result_response;

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
        amr_mqtt_bridge_publish_navigate_response(
          g_amr_mqtt_bridge_ros_state.navigate_state.request_id,
          result_response.result.success,
          (int)result_response.status,
          true,
          true,
          result_response.result.message.data);
        memset(&g_amr_mqtt_bridge_ros_state.navigate_state, 0, sizeof(g_amr_mqtt_bridge_ros_state.navigate_state));
      }
      amr_msgs__action__NavigateToPose_GetResult_Response__fini(&result_response);
    }
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

    if (strcmp(topic_name, g_amr_mqtt_bridge_config.mqtt.command_cmd_vel) == 0) {
      amr_mqtt_bridge_handle_cmd_vel_message(
        message->payload,
        (size_t)message->payloadlen);
    } else if (strcmp(topic_name, g_amr_mqtt_bridge_config.mqtt.command_save_map) == 0) {
      char * payload_text = (char *)calloc((size_t)message->payloadlen + 1U, sizeof(char));
      if (payload_text != NULL) {
        memcpy(payload_text, message->payload, (size_t)message->payloadlen);
        amr_mqtt_bridge_handle_save_map_command(payload_text);
        free(payload_text);
      }
    } else if (strcmp(topic_name, g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose) == 0) {
      char * payload_text = (char *)calloc((size_t)message->payloadlen + 1U, sizeof(char));
      if (payload_text != NULL) {
        memcpy(payload_text, message->payload, (size_t)message->payloadlen);
        amr_mqtt_bridge_handle_set_initial_pose_command(payload_text);
        free(payload_text);
      }
    } else if (strcmp(topic_name, g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose) == 0) {
      char * payload_text = (char *)calloc((size_t)message->payloadlen + 1U, sizeof(char));
      if (payload_text != NULL) {
        memcpy(payload_text, message->payload, (size_t)message->payloadlen);
        amr_mqtt_bridge_handle_navigate_to_pose_command(payload_text);
        free(payload_text);
      }
    } else if (strcmp(topic_name, g_amr_mqtt_bridge_config.mqtt.command_cancel_navigate_to_pose) == 0) {
      char * payload_text = (char *)calloc((size_t)message->payloadlen + 1U, sizeof(char));
      if (payload_text != NULL) {
        memcpy(payload_text, message->payload, (size_t)message->payloadlen);
        amr_mqtt_bridge_handle_navigate_cancel_command(payload_text);
        free(payload_text);
      }
    } else if (strcmp(topic_name, g_amr_mqtt_bridge_config.mqtt.command_ping) == 0) {
      char * payload_text = (char *)calloc((size_t)message->payloadlen + 1U, sizeof(char));
      if (payload_text != NULL) {
        memcpy(payload_text, message->payload, (size_t)message->payloadlen);
        amr_mqtt_bridge_handle_ping_command(payload_text);
        free(payload_text);
      }
    } else if (strcmp(topic_name, g_amr_mqtt_bridge_config.mqtt.request_plan_segment) == 0) {
      char * payload_text = (char *)calloc((size_t)message->payloadlen + 1U, sizeof(char));
      if (payload_text != NULL) {
        memcpy(payload_text, message->payload, (size_t)message->payloadlen);
        amr_mqtt_bridge_handle_plan_segment_request(payload_text);
        free(payload_text);
      }
    } else if (strcmp(topic_name, g_amr_mqtt_bridge_config.mqtt.request_plan_route) == 0) {
      char * payload_text = (char *)calloc((size_t)message->payloadlen + 1U, sizeof(char));
      if (payload_text != NULL) {
        memcpy(payload_text, message->payload, (size_t)message->payloadlen);
        amr_mqtt_bridge_handle_plan_route_request(payload_text);
        free(payload_text);
      }
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topic_name);
    ++processed_count;
  } while (processed_count < 8);
}

rcl_ret_t amr_mqtt_bridge_initialize(int argc, const char *argv[])
{
  rcl_ret_t rc;

  g_amr_mqtt_bridge_runtime.allocator = rcl_get_default_allocator();
  (void)memset(&g_amr_mqtt_bridge_runtime.support, 0, sizeof(g_amr_mqtt_bridge_runtime.support));
  g_amr_mqtt_bridge_runtime.node = rcl_get_zero_initialized_node();
  g_amr_mqtt_bridge_runtime.executor = rclc_executor_get_zero_initialized_executor();
  g_amr_mqtt_bridge_runtime.return_code = 0;
  amr_mqtt_bridge_set_default_config();

  rc = rclc_support_init(
    &g_amr_mqtt_bridge_runtime.support,
    argc,
    argv,
    &g_amr_mqtt_bridge_runtime.allocator);
  if (rc != RCL_RET_OK) {
    return rc;
  }

  amr_mqtt_bridge_load_parameter_overrides();
  rc = rclc_node_init_default(
    &g_amr_mqtt_bridge_runtime.node,
    AMR_MQTT_BRIDGE_NODE_NAME,
    AMR_MQTT_BRIDGE_NODE_NAMESPACE,
    &g_amr_mqtt_bridge_runtime.support);
  if (rc != RCL_RET_OK) {
    return rc;
  }

  rc = rclc_executor_init(
    &g_amr_mqtt_bridge_runtime.executor,
    &g_amr_mqtt_bridge_runtime.support.context,
    AMR_MQTT_BRIDGE_MAX_TELEMETRY_ENDPOINTS,
    &g_amr_mqtt_bridge_runtime.allocator);
  if (rc != RCL_RET_OK) {
    return rc;
  }

  if (amr_mqtt_bridge_connect_mqtt() != 0 ||
    amr_mqtt_bridge_init_ros_interfaces() != 0)
  {
    return RCL_RET_ERROR;
  }

  g_amr_mqtt_bridge_runtime.is_initialized = true;
  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_bridge",
    "Initialized MQTT robot plugin with broker %s:%d",
    g_amr_mqtt_bridge_config.broker.host,
    g_amr_mqtt_bridge_config.broker.port);
  return RCL_RET_OK;
}

void amr_mqtt_bridge_run(void)
{
  if (!g_amr_mqtt_bridge_runtime.is_initialized) {
    return;
  }

  while (rcl_context_is_valid(&g_amr_mqtt_bridge_runtime.support.context)) {
    rcl_ret_t rc = rclc_executor_spin_some(
      &g_amr_mqtt_bridge_runtime.executor,
      20 * 1000 * 1000);
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
      rcl_reset_error();
      g_amr_mqtt_bridge_runtime.return_code = 1;
      break;
    }
    amr_mqtt_bridge_poll_mqtt();
    amr_mqtt_bridge_poll_navigate_action();
  }
}

rcl_ret_t amr_mqtt_bridge_terminate(void)
{
  if (!g_amr_mqtt_bridge_runtime.is_initialized) {
    return (rcl_ret_t)g_amr_mqtt_bridge_runtime.return_code;
  }

  amr_mqtt_bridge_fini_ros_interfaces();
  amr_mqtt_bridge_disconnect_mqtt();
  amr_mqtt_bridge_log_rcl_error(
    "executor",
    rclc_executor_fini(&g_amr_mqtt_bridge_runtime.executor));
  amr_mqtt_bridge_log_rcl_error(
    "node",
    rcl_node_fini(&g_amr_mqtt_bridge_runtime.node));
  amr_mqtt_bridge_log_rcl_error(
    "support",
    rclc_support_fini(&g_amr_mqtt_bridge_runtime.support));
  g_amr_mqtt_bridge_runtime.is_initialized = false;
  return (rcl_ret_t)g_amr_mqtt_bridge_runtime.return_code;
}
