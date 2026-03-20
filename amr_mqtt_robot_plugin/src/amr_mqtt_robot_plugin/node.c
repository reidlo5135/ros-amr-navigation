#include "amr_mqtt_robot_plugin/node.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <rcl/arguments.h>
#include <rcl/error_handling.h>
#include <rcl/publisher.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rcutils/error_handling.h>
#include <rcutils/logging.h>
#include <rcutils/logging_macros.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/detail/twist__functions.h>
#include <geometry_msgs/msg/detail/twist__type_support.h>
#include <nav_msgs/msg/detail/odometry__functions.h>
#include <nav_msgs/msg/detail/odometry__type_support.h>
#include <sensor_msgs/msg/detail/imu__functions.h>
#include <sensor_msgs/msg/detail/imu__type_support.h>
#include <sensor_msgs/msg/detail/joint_state__functions.h>
#include <sensor_msgs/msg/detail/joint_state__type_support.h>
#include <sensor_msgs/msg/detail/laser_scan__functions.h>
#include <sensor_msgs/msg/detail/laser_scan__type_support.h>
#include <tf2_msgs/msg/detail/tf_message__functions.h>
#include <tf2_msgs/msg/detail/tf_message__type_support.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rcl_yaml_param_parser/parser.h>

amr_mqtt_robot_plugin_runtime_t g_amr_mqtt_robot_plugin_runtime = {0};
amr_mqtt_robot_plugin_mqtt_state_t g_amr_mqtt_robot_plugin_mqtt = {0};
amr_mqtt_robot_plugin_config_t g_amr_mqtt_robot_plugin_config = {0};
amr_mqtt_robot_plugin_ros_state_t g_amr_mqtt_robot_plugin_ros_state = {0};

typedef struct amr_mqtt_robot_plugin_string_builder_s
{
  char * data;
  size_t length;
  size_t capacity;
} amr_mqtt_robot_plugin_string_builder_t;

static const char * k_node_name = "/amr/mqtt_robot_plugin";
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

static void amr_mqtt_robot_plugin_log_rcl_error(const char * label, rcl_ret_t rc)
{
  if (rc == RCL_RET_OK) {
    return;
  }
  RCUTILS_LOG_ERROR_NAMED(
    "amr_mqtt_robot_plugin",
    "Failed to finalize %s: %s",
    label,
    rcl_get_error_string().str);
  rcl_reset_error();
  g_amr_mqtt_robot_plugin_runtime.return_code = 1;
}

static void amr_mqtt_robot_plugin_copy_string(char * destination, size_t capacity, const char * source)
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

static void amr_mqtt_robot_plugin_set_default_config(void)
{
  memset(&g_amr_mqtt_robot_plugin_config, 0, sizeof(g_amr_mqtt_robot_plugin_config));

  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.broker.host,
    sizeof(g_amr_mqtt_robot_plugin_config.broker.host),
    "192.168.61.35");
  g_amr_mqtt_robot_plugin_config.broker.port = 1883;
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.broker.client_id,
    sizeof(g_amr_mqtt_robot_plugin_config.broker.client_id),
    "amr_mqtt_robot_plugin");
  g_amr_mqtt_robot_plugin_config.broker.keep_alive_sec = 20;
  g_amr_mqtt_robot_plugin_config.broker.clean_session = true;

  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.mqtt.root,
    sizeof(g_amr_mqtt_robot_plugin_config.mqtt.root),
    "amr/robot/turtlebot3");
  g_amr_mqtt_robot_plugin_config.mqtt.telemetry_qos = 0;
  g_amr_mqtt_robot_plugin_config.mqtt.command_qos = 1;
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_scan,
    sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_scan),
    "amr/robot/turtlebot3/telemetry/scan");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_odom,
    sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_odom),
    "amr/robot/turtlebot3/telemetry/odom");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_imu,
    sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_imu),
    "amr/robot/turtlebot3/telemetry/imu");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf,
    sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf),
    "amr/robot/turtlebot3/telemetry/tf");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf_static,
    sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf_static),
    "amr/robot/turtlebot3/telemetry/tf_static");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_joint_states,
    sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_joint_states),
    "amr/robot/turtlebot3/telemetry/joint_states");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.mqtt.command_cmd_vel,
    sizeof(g_amr_mqtt_robot_plugin_config.mqtt.command_cmd_vel),
    "amr/robot/turtlebot3/command/cmd_vel");

  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.ros.topic_scan,
    sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_scan),
    "/scan");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.ros.topic_odom,
    sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_odom),
    "/odom");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.ros.topic_imu,
    sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_imu),
    "/imu");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.ros.topic_tf,
    sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_tf),
    "/tf");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.ros.topic_tf_static,
    sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_tf_static),
    "/tf_static");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.ros.topic_joint_states,
    sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_joint_states),
    "/joint_states");
  amr_mqtt_robot_plugin_copy_string(
    g_amr_mqtt_robot_plugin_config.ros.topic_cmd_vel,
    sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_cmd_vel),
    "/cmd_vel");
}

static const rcl_node_params_t * amr_mqtt_robot_plugin_find_node_params(
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

static const rcl_variant_t * amr_mqtt_robot_plugin_find_param_variant(
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

static void amr_mqtt_robot_plugin_read_string_param(
  const rcl_node_params_t * node_params,
  const char * parameter_name,
  char * destination,
  size_t capacity)
{
  const rcl_variant_t * variant =
    amr_mqtt_robot_plugin_find_param_variant(node_params, parameter_name);
  if (variant == NULL || variant->string_value == NULL) {
    return;
  }
  amr_mqtt_robot_plugin_copy_string(destination, capacity, variant->string_value);
}

static void amr_mqtt_robot_plugin_read_integer_param(
  const rcl_node_params_t * node_params,
  const char * parameter_name,
  int * destination)
{
  const rcl_variant_t * variant =
    amr_mqtt_robot_plugin_find_param_variant(node_params, parameter_name);
  if (variant == NULL || variant->integer_value == NULL || destination == NULL) {
    return;
  }
  *destination = (int)(*variant->integer_value);
}

static void amr_mqtt_robot_plugin_read_bool_param(
  const rcl_node_params_t * node_params,
  const char * parameter_name,
  bool * destination)
{
  const rcl_variant_t * variant =
    amr_mqtt_robot_plugin_find_param_variant(node_params, parameter_name);
  if (variant == NULL || variant->bool_value == NULL || destination == NULL) {
    return;
  }
  *destination = *variant->bool_value;
}

static void amr_mqtt_robot_plugin_load_parameter_overrides(void)
{
  rcl_params_t * parameter_overrides = NULL;
  const rcl_node_params_t * node_params = NULL;
  rcl_ret_t rc = rcl_arguments_get_param_overrides(
    &g_amr_mqtt_robot_plugin_runtime.support.context.global_arguments,
    &parameter_overrides);
  if (rc != RCL_RET_OK || parameter_overrides == NULL) {
    if (rc != RCL_RET_OK) {
      rcl_reset_error();
    }
    return;
  }

  node_params = amr_mqtt_robot_plugin_find_node_params(parameter_overrides, k_node_name);
  if (node_params != NULL) {
    amr_mqtt_robot_plugin_read_string_param(node_params, "broker.host", g_amr_mqtt_robot_plugin_config.broker.host, sizeof(g_amr_mqtt_robot_plugin_config.broker.host));
    amr_mqtt_robot_plugin_read_integer_param(node_params, "broker.port", &g_amr_mqtt_robot_plugin_config.broker.port);
    amr_mqtt_robot_plugin_read_string_param(node_params, "broker.client_id", g_amr_mqtt_robot_plugin_config.broker.client_id, sizeof(g_amr_mqtt_robot_plugin_config.broker.client_id));
    amr_mqtt_robot_plugin_read_integer_param(node_params, "broker.keep_alive_sec", &g_amr_mqtt_robot_plugin_config.broker.keep_alive_sec);
    amr_mqtt_robot_plugin_read_bool_param(node_params, "broker.clean_session", &g_amr_mqtt_robot_plugin_config.broker.clean_session);
    amr_mqtt_robot_plugin_read_string_param(node_params, "broker.username", g_amr_mqtt_robot_plugin_config.broker.username, sizeof(g_amr_mqtt_robot_plugin_config.broker.username));
    amr_mqtt_robot_plugin_read_string_param(node_params, "broker.password", g_amr_mqtt_robot_plugin_config.broker.password, sizeof(g_amr_mqtt_robot_plugin_config.broker.password));

    amr_mqtt_robot_plugin_read_string_param(node_params, "mqtt.root", g_amr_mqtt_robot_plugin_config.mqtt.root, sizeof(g_amr_mqtt_robot_plugin_config.mqtt.root));
    amr_mqtt_robot_plugin_read_integer_param(node_params, "mqtt.qos.telemetry", &g_amr_mqtt_robot_plugin_config.mqtt.telemetry_qos);
    amr_mqtt_robot_plugin_read_integer_param(node_params, "mqtt.qos.command", &g_amr_mqtt_robot_plugin_config.mqtt.command_qos);
    amr_mqtt_robot_plugin_read_string_param(node_params, "mqtt.topics.telemetry.scan", g_amr_mqtt_robot_plugin_config.mqtt.telemetry_scan, sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_scan));
    amr_mqtt_robot_plugin_read_string_param(node_params, "mqtt.topics.telemetry.odom", g_amr_mqtt_robot_plugin_config.mqtt.telemetry_odom, sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_odom));
    amr_mqtt_robot_plugin_read_string_param(node_params, "mqtt.topics.telemetry.imu", g_amr_mqtt_robot_plugin_config.mqtt.telemetry_imu, sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_imu));
    amr_mqtt_robot_plugin_read_string_param(node_params, "mqtt.topics.telemetry.tf", g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf, sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf));
    amr_mqtt_robot_plugin_read_string_param(node_params, "mqtt.topics.telemetry.tf_static", g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf_static, sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf_static));
    amr_mqtt_robot_plugin_read_string_param(node_params, "mqtt.topics.telemetry.joint_states", g_amr_mqtt_robot_plugin_config.mqtt.telemetry_joint_states, sizeof(g_amr_mqtt_robot_plugin_config.mqtt.telemetry_joint_states));
    amr_mqtt_robot_plugin_read_string_param(node_params, "mqtt.topics.command.cmd_vel", g_amr_mqtt_robot_plugin_config.mqtt.command_cmd_vel, sizeof(g_amr_mqtt_robot_plugin_config.mqtt.command_cmd_vel));

    amr_mqtt_robot_plugin_read_string_param(node_params, "ros.topics.scan", g_amr_mqtt_robot_plugin_config.ros.topic_scan, sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_scan));
    amr_mqtt_robot_plugin_read_string_param(node_params, "ros.topics.odom", g_amr_mqtt_robot_plugin_config.ros.topic_odom, sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_odom));
    amr_mqtt_robot_plugin_read_string_param(node_params, "ros.topics.imu", g_amr_mqtt_robot_plugin_config.ros.topic_imu, sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_imu));
    amr_mqtt_robot_plugin_read_string_param(node_params, "ros.topics.tf", g_amr_mqtt_robot_plugin_config.ros.topic_tf, sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_tf));
    amr_mqtt_robot_plugin_read_string_param(node_params, "ros.topics.tf_static", g_amr_mqtt_robot_plugin_config.ros.topic_tf_static, sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_tf_static));
    amr_mqtt_robot_plugin_read_string_param(node_params, "ros.topics.joint_states", g_amr_mqtt_robot_plugin_config.ros.topic_joint_states, sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_joint_states));
    amr_mqtt_robot_plugin_read_string_param(node_params, "ros.topics.cmd_vel", g_amr_mqtt_robot_plugin_config.ros.topic_cmd_vel, sizeof(g_amr_mqtt_robot_plugin_config.ros.topic_cmd_vel));
  }

  rcl_yaml_node_struct_fini(parameter_overrides);
}

static int amr_mqtt_robot_plugin_subscribe_command_topics(void)
{
  int mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_robot_plugin_mqtt.client,
    g_amr_mqtt_robot_plugin_config.mqtt.command_cmd_vel,
    g_amr_mqtt_robot_plugin_config.mqtt.command_qos);
  if (mqtt_rc == MQTTCLIENT_SUCCESS) {
    g_amr_mqtt_robot_plugin_mqtt.command_subscriptions_registered = true;
    RCUTILS_LOG_INFO_NAMED(
      "amr_mqtt_robot_plugin",
      "Subscribed MQTT command topic '%s' (qos=%d)",
      g_amr_mqtt_robot_plugin_config.mqtt.command_cmd_vel,
      g_amr_mqtt_robot_plugin_config.mqtt.command_qos);
  } else {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_robot_plugin",
      "Failed to subscribe MQTT command topic '%s': rc=%d (%s)",
      g_amr_mqtt_robot_plugin_config.mqtt.command_cmd_vel,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
  }
  return mqtt_rc;
}

static int amr_mqtt_robot_plugin_connect_mqtt(void)
{
  int mqtt_rc = 0;
  int broker_uri_length = snprintf(
    g_amr_mqtt_robot_plugin_mqtt.broker_uri,
    sizeof(g_amr_mqtt_robot_plugin_mqtt.broker_uri),
    "tcp://%s:%d",
    g_amr_mqtt_robot_plugin_config.broker.host,
    g_amr_mqtt_robot_plugin_config.broker.port);
  MQTTClient_connectOptions connect_options = MQTTClient_connectOptions_initializer;

  if (broker_uri_length < 0 ||
    (size_t)broker_uri_length >= sizeof(g_amr_mqtt_robot_plugin_mqtt.broker_uri))
  {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_robot_plugin",
      "MQTT broker URI is too long for host '%s' and port %d",
      g_amr_mqtt_robot_plugin_config.broker.host,
      g_amr_mqtt_robot_plugin_config.broker.port);
    return 1;
  }

  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_robot_plugin",
    "Connecting to MQTT broker at %s",
    g_amr_mqtt_robot_plugin_mqtt.broker_uri);

  mqtt_rc = MQTTClient_create(
    &g_amr_mqtt_robot_plugin_mqtt.client,
    g_amr_mqtt_robot_plugin_mqtt.broker_uri,
    g_amr_mqtt_robot_plugin_config.broker.client_id,
    MQTTCLIENT_PERSISTENCE_NONE,
    NULL);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_robot_plugin",
      "Failed to create MQTT client for %s: rc=%d (%s)",
      g_amr_mqtt_robot_plugin_mqtt.broker_uri,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
    return 1;
  }

  g_amr_mqtt_robot_plugin_mqtt.client_created = true;
  connect_options.keepAliveInterval = g_amr_mqtt_robot_plugin_config.broker.keep_alive_sec;
  connect_options.cleansession = g_amr_mqtt_robot_plugin_config.broker.clean_session ? 1 : 0;
  if (g_amr_mqtt_robot_plugin_config.broker.username[0] != '\0') {
    connect_options.username = g_amr_mqtt_robot_plugin_config.broker.username;
  }
  if (g_amr_mqtt_robot_plugin_config.broker.password[0] != '\0') {
    connect_options.password = g_amr_mqtt_robot_plugin_config.broker.password;
  }

  mqtt_rc = MQTTClient_connect(g_amr_mqtt_robot_plugin_mqtt.client, &connect_options);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_robot_plugin",
      "Failed to connect MQTT broker %s: rc=%d (%s)",
      g_amr_mqtt_robot_plugin_mqtt.broker_uri,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
    MQTTClient_destroy(&g_amr_mqtt_robot_plugin_mqtt.client);
    g_amr_mqtt_robot_plugin_mqtt.client_created = false;
    return 1;
  }

  g_amr_mqtt_robot_plugin_mqtt.connected = true;
  g_amr_mqtt_robot_plugin_mqtt.command_subscriptions_registered = false;
  g_amr_mqtt_robot_plugin_mqtt.last_reconnect_attempt_sec = 0;
  mqtt_rc = amr_mqtt_robot_plugin_subscribe_command_topics();
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return 1;
  }

  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_robot_plugin",
    "Connected to MQTT broker at %s",
    g_amr_mqtt_robot_plugin_mqtt.broker_uri);
  return 0;
}

static void amr_mqtt_robot_plugin_disconnect_mqtt(void)
{
  if (g_amr_mqtt_robot_plugin_mqtt.connected) {
    (void)MQTTClient_disconnect(g_amr_mqtt_robot_plugin_mqtt.client, 1000);
    g_amr_mqtt_robot_plugin_mqtt.connected = false;
  }
  g_amr_mqtt_robot_plugin_mqtt.command_subscriptions_registered = false;
  if (g_amr_mqtt_robot_plugin_mqtt.client_created) {
    MQTTClient_destroy(&g_amr_mqtt_robot_plugin_mqtt.client);
    g_amr_mqtt_robot_plugin_mqtt.client_created = false;
  }
}

static bool amr_mqtt_robot_plugin_ensure_connected(void)
{
  long now_sec = (long)time(NULL);
  int mqtt_rc = 0;
  MQTTClient_connectOptions connect_options = MQTTClient_connectOptions_initializer;

  if (!g_amr_mqtt_robot_plugin_mqtt.client_created) {
    return false;
  }
  if (MQTTClient_isConnected(g_amr_mqtt_robot_plugin_mqtt.client)) {
    g_amr_mqtt_robot_plugin_mqtt.connected = true;
    return true;
  }

  if (now_sec == g_amr_mqtt_robot_plugin_mqtt.last_reconnect_attempt_sec) {
    return false;
  }

  g_amr_mqtt_robot_plugin_mqtt.last_reconnect_attempt_sec = now_sec;
  connect_options.keepAliveInterval = g_amr_mqtt_robot_plugin_config.broker.keep_alive_sec;
  connect_options.cleansession = g_amr_mqtt_robot_plugin_config.broker.clean_session ? 1 : 0;
  if (g_amr_mqtt_robot_plugin_config.broker.username[0] != '\0') {
    connect_options.username = g_amr_mqtt_robot_plugin_config.broker.username;
  }
  if (g_amr_mqtt_robot_plugin_config.broker.password[0] != '\0') {
    connect_options.password = g_amr_mqtt_robot_plugin_config.broker.password;
  }
  mqtt_rc = MQTTClient_connect(g_amr_mqtt_robot_plugin_mqtt.client, &connect_options);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    RCUTILS_LOG_WARN_NAMED(
      "amr_mqtt_robot_plugin",
      "Failed to reconnect MQTT broker %s: rc=%d (%s)",
      g_amr_mqtt_robot_plugin_mqtt.broker_uri,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
    return false;
  }

  g_amr_mqtt_robot_plugin_mqtt.connected = true;
  if (!g_amr_mqtt_robot_plugin_mqtt.command_subscriptions_registered) {
    mqtt_rc = amr_mqtt_robot_plugin_subscribe_command_topics();
    if (mqtt_rc != MQTTCLIENT_SUCCESS) {
      g_amr_mqtt_robot_plugin_mqtt.connected = false;
      return false;
    }
  }
  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_robot_plugin",
    "Reconnected to MQTT broker at %s",
    g_amr_mqtt_robot_plugin_mqtt.broker_uri);
  return true;
}

static bool amr_mqtt_robot_plugin_builder_init(
  amr_mqtt_robot_plugin_string_builder_t * builder,
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

static void amr_mqtt_robot_plugin_builder_fini(amr_mqtt_robot_plugin_string_builder_t * builder)
{
  if (builder->data != NULL) {
    free(builder->data);
  }
  builder->data = NULL;
  builder->length = 0U;
  builder->capacity = 0U;
}

static bool amr_mqtt_robot_plugin_builder_reserve(
  amr_mqtt_robot_plugin_string_builder_t * builder,
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

static bool amr_mqtt_robot_plugin_builder_append(
  amr_mqtt_robot_plugin_string_builder_t * builder,
  const char * text)
{
  size_t text_length = strlen(text);
  if (!amr_mqtt_robot_plugin_builder_reserve(builder, text_length)) {
    return false;
  }
  memcpy(builder->data + builder->length, text, text_length + 1U);
  builder->length += text_length;
  return true;
}

static bool amr_mqtt_robot_plugin_builder_appendf(
  amr_mqtt_robot_plugin_string_builder_t * builder,
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
  if (!amr_mqtt_robot_plugin_builder_reserve(builder, (size_t)required)) {
    va_end(args);
    return false;
  }
  (void)vsnprintf(builder->data + builder->length, builder->capacity - builder->length, format, args);
  builder->length += (size_t)required;
  va_end(args);
  return true;
}

static bool amr_mqtt_robot_plugin_builder_append_json_string(
  amr_mqtt_robot_plugin_string_builder_t * builder,
  const char * text)
{
  const char * safe_text = text != NULL ? text : "";
  if (!amr_mqtt_robot_plugin_builder_append(builder, "\"")) {
    return false;
  }
  for (size_t index = 0; safe_text[index] != '\0'; ++index) {
    char current = safe_text[index];
    if (current == '\\') {
      if (!amr_mqtt_robot_plugin_builder_append(builder, "\\\\")) {
        return false;
      }
    } else if (current == '"') {
      if (!amr_mqtt_robot_plugin_builder_append(builder, "\\\"")) {
        return false;
      }
    } else {
      char buffer[2] = {current, '\0'};
      if (!amr_mqtt_robot_plugin_builder_append(builder, buffer)) {
        return false;
      }
    }
  }
  return amr_mqtt_robot_plugin_builder_append(builder, "\"");
}

static char * amr_mqtt_robot_plugin_builder_take(
  amr_mqtt_robot_plugin_string_builder_t * builder)
{
  char * data = builder->data;
  builder->data = NULL;
  builder->length = 0U;
  builder->capacity = 0U;
  return data;
}

static bool amr_mqtt_robot_plugin_append_header(
  amr_mqtt_robot_plugin_string_builder_t * builder,
  const std_msgs__msg__Header * header)
{
  return amr_mqtt_robot_plugin_builder_appendf(
    builder,
    "\"header\":{\"stamp\":{\"sec\":%d,\"nanosec\":%u},\"frame_id\":",
    header->stamp.sec,
    header->stamp.nanosec) &&
    amr_mqtt_robot_plugin_builder_append_json_string(builder, header->frame_id.data) &&
    amr_mqtt_robot_plugin_builder_append(builder, "}");
}

static char * amr_mqtt_robot_plugin_serialize_scan(const void * message)
{
  const sensor_msgs__msg__LaserScan * scan = (const sensor_msgs__msg__LaserScan *)message;
  amr_mqtt_robot_plugin_string_builder_t builder = {0};

  if (!amr_mqtt_robot_plugin_builder_init(&builder, 1024U) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, "{") ||
    !amr_mqtt_robot_plugin_append_header(&builder, &scan->header) ||
    !amr_mqtt_robot_plugin_builder_appendf(
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
    amr_mqtt_robot_plugin_builder_fini(&builder);
    return NULL;
  }

  for (size_t index = 0; index < scan->ranges.size; ++index) {
    if (index > 0U && !amr_mqtt_robot_plugin_builder_append(&builder, ",")) {
      amr_mqtt_robot_plugin_builder_fini(&builder);
      return NULL;
    }
    if (!amr_mqtt_robot_plugin_builder_appendf(&builder, "%.6f", scan->ranges.data[index])) {
      amr_mqtt_robot_plugin_builder_fini(&builder);
      return NULL;
    }
  }

  if (!amr_mqtt_robot_plugin_builder_append(&builder, "]}")) {
    amr_mqtt_robot_plugin_builder_fini(&builder);
    return NULL;
  }
  return amr_mqtt_robot_plugin_builder_take(&builder);
}

static char * amr_mqtt_robot_plugin_serialize_odom(const void * message)
{
  const nav_msgs__msg__Odometry * odom = (const nav_msgs__msg__Odometry *)message;
  amr_mqtt_robot_plugin_string_builder_t builder = {0};

  if (!amr_mqtt_robot_plugin_builder_init(&builder, 512U) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, "{") ||
    !amr_mqtt_robot_plugin_append_header(&builder, &odom->header) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, ",\"child_frame_id\":") ||
    !amr_mqtt_robot_plugin_builder_append_json_string(&builder, odom->child_frame_id.data) ||
    !amr_mqtt_robot_plugin_builder_appendf(
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
    amr_mqtt_robot_plugin_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_robot_plugin_builder_take(&builder);
}

static char * amr_mqtt_robot_plugin_serialize_imu(const void * message)
{
  const sensor_msgs__msg__Imu * imu = (const sensor_msgs__msg__Imu *)message;
  amr_mqtt_robot_plugin_string_builder_t builder = {0};

  if (!amr_mqtt_robot_plugin_builder_init(&builder, 512U) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, "{") ||
    !amr_mqtt_robot_plugin_append_header(&builder, &imu->header) ||
    !amr_mqtt_robot_plugin_builder_appendf(
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
    amr_mqtt_robot_plugin_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_robot_plugin_builder_take(&builder);
}

static bool amr_mqtt_robot_plugin_builder_append_double_array(
  amr_mqtt_robot_plugin_string_builder_t * builder,
  const double * values,
  size_t count)
{
  if (!amr_mqtt_robot_plugin_builder_append(builder, "[")) {
    return false;
  }

  for (size_t index = 0; index < count; ++index) {
    if (index > 0U && !amr_mqtt_robot_plugin_builder_append(builder, ",")) {
      return false;
    }
    if (!amr_mqtt_robot_plugin_builder_appendf(builder, "%.6f", values[index])) {
      return false;
    }
  }

  return amr_mqtt_robot_plugin_builder_append(builder, "]");
}

static bool amr_mqtt_robot_plugin_builder_append_string_array(
  amr_mqtt_robot_plugin_string_builder_t * builder,
  const rosidl_runtime_c__String__Sequence * values)
{
  if (!amr_mqtt_robot_plugin_builder_append(builder, "[")) {
    return false;
  }

  for (size_t index = 0; index < values->size; ++index) {
    if (index > 0U && !amr_mqtt_robot_plugin_builder_append(builder, ",")) {
      return false;
    }
    if (!amr_mqtt_robot_plugin_builder_append_json_string(builder, values->data[index].data)) {
      return false;
    }
  }

  return amr_mqtt_robot_plugin_builder_append(builder, "]");
}

static bool amr_mqtt_robot_plugin_append_transform_stamped(
  amr_mqtt_robot_plugin_string_builder_t * builder,
  const geometry_msgs__msg__TransformStamped * transform)
{
  return amr_mqtt_robot_plugin_builder_append(builder, "{") &&
    amr_mqtt_robot_plugin_append_header(builder, &transform->header) &&
    amr_mqtt_robot_plugin_builder_append(builder, ",\"child_frame_id\":") &&
    amr_mqtt_robot_plugin_builder_append_json_string(builder, transform->child_frame_id.data) &&
    amr_mqtt_robot_plugin_builder_appendf(
      builder,
      ",\"translation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
      "\"rotation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f}}",
      transform->transform.translation.x,
      transform->transform.translation.y,
      transform->transform.translation.z,
      transform->transform.rotation.x,
      transform->transform.rotation.y,
      transform->transform.rotation.z,
      transform->transform.rotation.w);
}

static char * amr_mqtt_robot_plugin_serialize_tf_message(const void * message)
{
  const tf2_msgs__msg__TFMessage * tf_message = (const tf2_msgs__msg__TFMessage *)message;
  amr_mqtt_robot_plugin_string_builder_t builder = {0};

  if (!amr_mqtt_robot_plugin_builder_init(&builder, 1024U) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, "{\"transforms\":["))
  {
    amr_mqtt_robot_plugin_builder_fini(&builder);
    return NULL;
  }

  for (size_t index = 0; index < tf_message->transforms.size; ++index) {
    if (index > 0U && !amr_mqtt_robot_plugin_builder_append(&builder, ",")) {
      amr_mqtt_robot_plugin_builder_fini(&builder);
      return NULL;
    }
    if (!amr_mqtt_robot_plugin_append_transform_stamped(&builder, &tf_message->transforms.data[index])) {
      amr_mqtt_robot_plugin_builder_fini(&builder);
      return NULL;
    }
  }

  if (!amr_mqtt_robot_plugin_builder_append(&builder, "]}")) {
    amr_mqtt_robot_plugin_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_robot_plugin_builder_take(&builder);
}

static char * amr_mqtt_robot_plugin_serialize_joint_states(const void * message)
{
  const sensor_msgs__msg__JointState * joint_states = (const sensor_msgs__msg__JointState *)message;
  amr_mqtt_robot_plugin_string_builder_t builder = {0};

  if (!amr_mqtt_robot_plugin_builder_init(&builder, 512U) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, "{") ||
    !amr_mqtt_robot_plugin_append_header(&builder, &joint_states->header) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, ",\"name\":") ||
    !amr_mqtt_robot_plugin_builder_append_string_array(&builder, &joint_states->name) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, ",\"position\":") ||
    !amr_mqtt_robot_plugin_builder_append_double_array(&builder, joint_states->position.data, joint_states->position.size) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, ",\"velocity\":") ||
    !amr_mqtt_robot_plugin_builder_append_double_array(&builder, joint_states->velocity.data, joint_states->velocity.size) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, ",\"effort\":") ||
    !amr_mqtt_robot_plugin_builder_append_double_array(&builder, joint_states->effort.data, joint_states->effort.size) ||
    !amr_mqtt_robot_plugin_builder_append(&builder, "}"))
  {
    amr_mqtt_robot_plugin_builder_fini(&builder);
    return NULL;
  }

  return amr_mqtt_robot_plugin_builder_take(&builder);
}

static bool amr_mqtt_robot_plugin_publish_payload(
  const char * mqtt_topic,
  const char * payload,
  int qos,
  bool retained)
{
  MQTTClient_message message = MQTTClient_message_initializer;
  MQTTClient_deliveryToken token = 0;
  int mqtt_rc = 0;

  if (!amr_mqtt_robot_plugin_ensure_connected()) {
    return false;
  }

  message.payload = (void *)payload;
  message.payloadlen = (int)strlen(payload);
  message.qos = qos;
  message.retained = retained ? 1 : 0;

  mqtt_rc = MQTTClient_publishMessage(
    g_amr_mqtt_robot_plugin_mqtt.client,
    mqtt_topic,
    &message,
    &token);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    if (mqtt_rc == MQTTCLIENT_DISCONNECTED) {
      g_amr_mqtt_robot_plugin_mqtt.connected = false;
    }
    return false;
  }

  if (qos > 0) {
    (void)MQTTClient_waitForCompletion(g_amr_mqtt_robot_plugin_mqtt.client, token, 1000L);
  }
  return true;
}

static bool amr_mqtt_robot_plugin_publish_binary_payload(
  const char * mqtt_topic,
  const void * payload,
  size_t payload_length,
  int qos,
  bool retained)
{
  MQTTClient_message message = MQTTClient_message_initializer;
  MQTTClient_deliveryToken token = 0;
  int mqtt_rc = 0;

  if (!amr_mqtt_robot_plugin_ensure_connected()) {
    return false;
  }

  message.payload = (void *)payload;
  message.payloadlen = (int)payload_length;
  message.qos = qos;
  message.retained = retained ? 1 : 0;

  mqtt_rc = MQTTClient_publishMessage(
    g_amr_mqtt_robot_plugin_mqtt.client,
    mqtt_topic,
    &message,
    &token);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    if (mqtt_rc == MQTTCLIENT_DISCONNECTED) {
      g_amr_mqtt_robot_plugin_mqtt.connected = false;
    }
    return false;
  }

  if (qos > 0) {
    (void)MQTTClient_waitForCompletion(g_amr_mqtt_robot_plugin_mqtt.client, token, 1000L);
  }
  return true;
}

static bool amr_mqtt_robot_plugin_serialize_message_raw(
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
      &g_amr_mqtt_robot_plugin_runtime.allocator) != RMW_RET_OK)
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

static bool amr_mqtt_robot_plugin_deserialize_twist_raw(
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
      &g_amr_mqtt_robot_plugin_runtime.allocator) != RMW_RET_OK)
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

static void amr_mqtt_robot_plugin_telemetry_callback(const void * message, void * context)
{
  const amr_mqtt_robot_plugin_telemetry_endpoint_t * endpoint =
    (const amr_mqtt_robot_plugin_telemetry_endpoint_t *)context;
  char * payload = NULL;
  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();

  if (message == NULL || endpoint == NULL) {
    return;
  }

  if (endpoint->raw_passthrough) {
    if (!amr_mqtt_robot_plugin_serialize_message_raw(
        message,
        endpoint->type_support,
        &serialized_message))
    {
      return;
    }
    (void)amr_mqtt_robot_plugin_publish_binary_payload(
      endpoint->mqtt_topic,
      serialized_message.buffer,
      serialized_message.buffer_length,
      endpoint->mqtt_qos,
      endpoint->retained);
    (void)rmw_serialized_message_fini(&serialized_message);
    return;
  }

  if (endpoint->serializer == NULL) {
    return;
  }

  payload = endpoint->serializer(message);
  if (payload == NULL) {
    return;
  }
  (void)amr_mqtt_robot_plugin_publish_payload(
    endpoint->mqtt_topic,
    payload,
    endpoint->mqtt_qos,
    endpoint->retained);
  free(payload);
}

static void amr_mqtt_robot_plugin_configure_endpoint(
  amr_mqtt_robot_plugin_telemetry_endpoint_t * endpoint,
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
  amr_mqtt_robot_plugin_serializer_fn_t serializer)
{
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
}

static int amr_mqtt_robot_plugin_add_subscription(
  amr_mqtt_robot_plugin_telemetry_endpoint_t * endpoint)
{
  rcl_ret_t rc = rclc_subscription_init(
    endpoint->subscription,
    &g_amr_mqtt_robot_plugin_runtime.node,
    endpoint->type_support,
    endpoint->ros_topic,
    endpoint->qos_profile);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }

  rc = rclc_executor_add_subscription_with_context(
    &g_amr_mqtt_robot_plugin_runtime.executor,
    endpoint->subscription,
    endpoint->message,
    amr_mqtt_robot_plugin_telemetry_callback,
    endpoint,
    ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }

  return 0;
}

static int amr_mqtt_robot_plugin_init_ros_interfaces(void)
{
  rcl_ret_t rc;

  memset(&g_amr_mqtt_robot_plugin_ros_state, 0, sizeof(g_amr_mqtt_robot_plugin_ros_state));
  if (!sensor_msgs__msg__LaserScan__init(&g_amr_mqtt_robot_plugin_ros_state.scan_message) ||
    !nav_msgs__msg__Odometry__init(&g_amr_mqtt_robot_plugin_ros_state.odom_message) ||
    !sensor_msgs__msg__Imu__init(&g_amr_mqtt_robot_plugin_ros_state.imu_message) ||
    !tf2_msgs__msg__TFMessage__init(&g_amr_mqtt_robot_plugin_ros_state.tf_message) ||
    !tf2_msgs__msg__TFMessage__init(&g_amr_mqtt_robot_plugin_ros_state.tf_static_message) ||
    !sensor_msgs__msg__JointState__init(&g_amr_mqtt_robot_plugin_ros_state.joint_states_message) ||
    !geometry_msgs__msg__Twist__init(&g_amr_mqtt_robot_plugin_ros_state.cmd_vel_message))
  {
    return 1;
  }
  g_amr_mqtt_robot_plugin_ros_state.messages_initialized = true;

  g_amr_mqtt_robot_plugin_ros_state.scan_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_robot_plugin_ros_state.odom_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_robot_plugin_ros_state.imu_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_robot_plugin_ros_state.tf_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_robot_plugin_ros_state.tf_static_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_robot_plugin_ros_state.joint_states_subscription = rcl_get_zero_initialized_subscription();
  g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoint_count = AMR_MQTT_ROBOT_PLUGIN_MAX_TELEMETRY_ENDPOINTS;

  amr_mqtt_robot_plugin_configure_endpoint(
    &g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[0],
    "scan",
    g_amr_mqtt_robot_plugin_config.ros.topic_scan,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_scan,
    &g_amr_mqtt_robot_plugin_ros_state.scan_subscription,
    &g_amr_mqtt_robot_plugin_ros_state.scan_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, sensor_msgs, msg, LaserScan)(),
    &k_sensor_qos,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_qos,
    false,
    true,
    NULL);
  amr_mqtt_robot_plugin_configure_endpoint(
    &g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[1],
    "odom",
    g_amr_mqtt_robot_plugin_config.ros.topic_odom,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_odom,
    &g_amr_mqtt_robot_plugin_ros_state.odom_subscription,
    &g_amr_mqtt_robot_plugin_ros_state.odom_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, nav_msgs, msg, Odometry)(),
    &k_sensor_qos,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_qos,
    false,
    true,
    NULL);
  amr_mqtt_robot_plugin_configure_endpoint(
    &g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[2],
    "imu",
    g_amr_mqtt_robot_plugin_config.ros.topic_imu,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_imu,
    &g_amr_mqtt_robot_plugin_ros_state.imu_subscription,
    &g_amr_mqtt_robot_plugin_ros_state.imu_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, sensor_msgs, msg, Imu)(),
    &k_sensor_qos,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_qos,
    false,
    true,
    NULL);
  amr_mqtt_robot_plugin_configure_endpoint(
    &g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[3],
    "tf",
    g_amr_mqtt_robot_plugin_config.ros.topic_tf,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf,
    &g_amr_mqtt_robot_plugin_ros_state.tf_subscription,
    &g_amr_mqtt_robot_plugin_ros_state.tf_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, tf2_msgs, msg, TFMessage)(),
    &k_default_qos,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_qos,
    false,
    true,
    NULL);
  amr_mqtt_robot_plugin_configure_endpoint(
    &g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[4],
    "tf_static",
    g_amr_mqtt_robot_plugin_config.ros.topic_tf_static,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_tf_static,
    &g_amr_mqtt_robot_plugin_ros_state.tf_static_subscription,
    &g_amr_mqtt_robot_plugin_ros_state.tf_static_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, tf2_msgs, msg, TFMessage)(),
    &k_transient_local_qos,
    g_amr_mqtt_robot_plugin_config.mqtt.command_qos,
    true,
    true,
    NULL);
  amr_mqtt_robot_plugin_configure_endpoint(
    &g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[5],
    "joint_states",
    g_amr_mqtt_robot_plugin_config.ros.topic_joint_states,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_joint_states,
    &g_amr_mqtt_robot_plugin_ros_state.joint_states_subscription,
    &g_amr_mqtt_robot_plugin_ros_state.joint_states_message,
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, sensor_msgs, msg, JointState)(),
    &k_default_qos,
    g_amr_mqtt_robot_plugin_config.mqtt.telemetry_qos,
    false,
    true,
    NULL);

  for (size_t index = 0; index < g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoint_count; ++index) {
    if (amr_mqtt_robot_plugin_add_subscription(&g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[index]) != 0) {
      return 1;
    }
  }
  g_amr_mqtt_robot_plugin_ros_state.subscriptions_initialized = true;

  g_amr_mqtt_robot_plugin_ros_state.cmd_vel_publisher = rcl_get_zero_initialized_publisher();
  rc = rclc_publisher_init_default(
    &g_amr_mqtt_robot_plugin_ros_state.cmd_vel_publisher,
    &g_amr_mqtt_robot_plugin_runtime.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    g_amr_mqtt_robot_plugin_config.ros.topic_cmd_vel);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return 1;
  }
  g_amr_mqtt_robot_plugin_ros_state.cmd_vel_publisher_initialized = true;
  return 0;
}

static void amr_mqtt_robot_plugin_fini_ros_interfaces(void)
{
  if (g_amr_mqtt_robot_plugin_ros_state.subscriptions_initialized) {
    for (size_t index = 0; index < g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoint_count; ++index) {
      if (g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[index].subscription != NULL) {
        amr_mqtt_robot_plugin_log_rcl_error(
          "subscription",
          rcl_subscription_fini(
            g_amr_mqtt_robot_plugin_ros_state.telemetry_endpoints[index].subscription,
            &g_amr_mqtt_robot_plugin_runtime.node));
      }
    }
    g_amr_mqtt_robot_plugin_ros_state.subscriptions_initialized = false;
  }

  if (g_amr_mqtt_robot_plugin_ros_state.cmd_vel_publisher_initialized) {
    amr_mqtt_robot_plugin_log_rcl_error(
      "cmd_vel publisher",
      rcl_publisher_fini(
        &g_amr_mqtt_robot_plugin_ros_state.cmd_vel_publisher,
        &g_amr_mqtt_robot_plugin_runtime.node));
    g_amr_mqtt_robot_plugin_ros_state.cmd_vel_publisher_initialized = false;
  }

  if (g_amr_mqtt_robot_plugin_ros_state.messages_initialized) {
    sensor_msgs__msg__LaserScan__fini(&g_amr_mqtt_robot_plugin_ros_state.scan_message);
    nav_msgs__msg__Odometry__fini(&g_amr_mqtt_robot_plugin_ros_state.odom_message);
    sensor_msgs__msg__Imu__fini(&g_amr_mqtt_robot_plugin_ros_state.imu_message);
    tf2_msgs__msg__TFMessage__fini(&g_amr_mqtt_robot_plugin_ros_state.tf_message);
    tf2_msgs__msg__TFMessage__fini(&g_amr_mqtt_robot_plugin_ros_state.tf_static_message);
    sensor_msgs__msg__JointState__fini(&g_amr_mqtt_robot_plugin_ros_state.joint_states_message);
    geometry_msgs__msg__Twist__fini(&g_amr_mqtt_robot_plugin_ros_state.cmd_vel_message);
    g_amr_mqtt_robot_plugin_ros_state.messages_initialized = false;
  }
}

static bool amr_mqtt_robot_plugin_extract_json_double(
  const char * payload,
  const char * key,
  double * output)
{
  char pattern[64] = {0};
  const char * key_pos = NULL;
  const char * cursor = NULL;
  char * parse_end = NULL;

  (void)snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  key_pos = strstr(payload, pattern);
  if (key_pos == NULL) {
    return false;
  }
  cursor = key_pos;
  while (*cursor != '\0' && *cursor != ':') {
    ++cursor;
  }
  if (*cursor != ':') {
    return false;
  }
  ++cursor;
  while (*cursor == ' ' || *cursor == '\n' || *cursor == '\r' || *cursor == '\t') {
    ++cursor;
  }
  *output = strtod(cursor, &parse_end);
  return parse_end != cursor;
}

static void amr_mqtt_robot_plugin_handle_cmd_vel_message(
  const void * payload,
  size_t payload_length)
{
  rcl_ret_t rc;

  if (!amr_mqtt_robot_plugin_deserialize_twist_raw(
      payload,
      payload_length,
      &g_amr_mqtt_robot_plugin_ros_state.cmd_vel_message))
  {
    return;
  }

  rc = rcl_publish(
    &g_amr_mqtt_robot_plugin_ros_state.cmd_vel_publisher,
    &g_amr_mqtt_robot_plugin_ros_state.cmd_vel_message,
    NULL);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
  }
}

static void amr_mqtt_robot_plugin_poll_mqtt(void)
{
  int mqtt_rc = 0;
  int processed_count = 0;

  if (!amr_mqtt_robot_plugin_ensure_connected()) {
    return;
  }

  do {
    char * topic_name = NULL;
    int topic_length = 0;
    MQTTClient_message * message = NULL;
    mqtt_rc = MQTTClient_receive(
      g_amr_mqtt_robot_plugin_mqtt.client,
      &topic_name,
      &topic_length,
      &message,
      0UL);
    if (mqtt_rc != MQTTCLIENT_SUCCESS) {
      if (mqtt_rc == MQTTCLIENT_DISCONNECTED) {
        g_amr_mqtt_robot_plugin_mqtt.connected = false;
      }
      break;
    }
    if (message == NULL || topic_name == NULL) {
      break;
    }

    if (strcmp(topic_name, g_amr_mqtt_robot_plugin_config.mqtt.command_cmd_vel) == 0) {
      amr_mqtt_robot_plugin_handle_cmd_vel_message(
        message->payload,
        (size_t)message->payloadlen);
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topic_name);
    ++processed_count;
  } while (processed_count < 8);
}

rcl_ret_t amr_mqtt_robot_plugin_initialize(int argc, const char *argv[])
{
  rcl_ret_t rc;

  g_amr_mqtt_robot_plugin_runtime.allocator = rcl_get_default_allocator();
  (void)memset(&g_amr_mqtt_robot_plugin_runtime.support, 0, sizeof(g_amr_mqtt_robot_plugin_runtime.support));
  g_amr_mqtt_robot_plugin_runtime.node = rcl_get_zero_initialized_node();
  g_amr_mqtt_robot_plugin_runtime.executor = rclc_executor_get_zero_initialized_executor();
  g_amr_mqtt_robot_plugin_runtime.return_code = 0;
  amr_mqtt_robot_plugin_set_default_config();

  rc = rclc_support_init(
    &g_amr_mqtt_robot_plugin_runtime.support,
    argc,
    argv,
    &g_amr_mqtt_robot_plugin_runtime.allocator);
  if (rc != RCL_RET_OK) {
    return rc;
  }

  amr_mqtt_robot_plugin_load_parameter_overrides();
  rc = rclc_node_init_default(
    &g_amr_mqtt_robot_plugin_runtime.node,
    AMR_MQTT_ROBOT_PLUGIN_NODE_NAME,
    AMR_MQTT_ROBOT_PLUGIN_NODE_NAMESPACE,
    &g_amr_mqtt_robot_plugin_runtime.support);
  if (rc != RCL_RET_OK) {
    return rc;
  }

  rc = rclc_executor_init(
    &g_amr_mqtt_robot_plugin_runtime.executor,
    &g_amr_mqtt_robot_plugin_runtime.support.context,
    AMR_MQTT_ROBOT_PLUGIN_MAX_TELEMETRY_ENDPOINTS,
    &g_amr_mqtt_robot_plugin_runtime.allocator);
  if (rc != RCL_RET_OK) {
    return rc;
  }

  if (amr_mqtt_robot_plugin_connect_mqtt() != 0 ||
    amr_mqtt_robot_plugin_init_ros_interfaces() != 0)
  {
    return RCL_RET_ERROR;
  }

  g_amr_mqtt_robot_plugin_runtime.is_initialized = true;
  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_robot_plugin",
    "Initialized MQTT robot plugin with broker %s:%d",
    g_amr_mqtt_robot_plugin_config.broker.host,
    g_amr_mqtt_robot_plugin_config.broker.port);
  return RCL_RET_OK;
}

void amr_mqtt_robot_plugin_run(void)
{
  if (!g_amr_mqtt_robot_plugin_runtime.is_initialized) {
    return;
  }

  while (rcl_context_is_valid(&g_amr_mqtt_robot_plugin_runtime.support.context)) {
    rcl_ret_t rc = rclc_executor_spin_some(
      &g_amr_mqtt_robot_plugin_runtime.executor,
      20 * 1000 * 1000);
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
      rcl_reset_error();
      g_amr_mqtt_robot_plugin_runtime.return_code = 1;
      break;
    }
    amr_mqtt_robot_plugin_poll_mqtt();
  }
}

rcl_ret_t amr_mqtt_robot_plugin_terminate(void)
{
  if (!g_amr_mqtt_robot_plugin_runtime.is_initialized) {
    return (rcl_ret_t)g_amr_mqtt_robot_plugin_runtime.return_code;
  }

  amr_mqtt_robot_plugin_fini_ros_interfaces();
  amr_mqtt_robot_plugin_disconnect_mqtt();
  amr_mqtt_robot_plugin_log_rcl_error(
    "executor",
    rclc_executor_fini(&g_amr_mqtt_robot_plugin_runtime.executor));
  amr_mqtt_robot_plugin_log_rcl_error(
    "node",
    rcl_node_fini(&g_amr_mqtt_robot_plugin_runtime.node));
  amr_mqtt_robot_plugin_log_rcl_error(
    "support",
    rclc_support_fini(&g_amr_mqtt_robot_plugin_runtime.support));
  g_amr_mqtt_robot_plugin_runtime.is_initialized = false;
  return (rcl_ret_t)g_amr_mqtt_robot_plugin_runtime.return_code;
}
