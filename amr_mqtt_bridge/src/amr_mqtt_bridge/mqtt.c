#include "amr_mqtt_bridge/mqtt.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include <MQTTClient.h>
#include <rcutils/logging_macros.h>

#include "amr_mqtt_bridge/node.h"

static int amr_mqtt_bridge_subscribe_command_topics(void)
{
  int mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.telemetry_robot_pose,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.telemetry_global_path,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.telemetry_local_path,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.telemetry_global_costmap,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.telemetry_local_costmap,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.telemetry_motion_status,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.telemetry_obstacle_report,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.feedback_navigate_to_pose,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.status_navigate_to_pose,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.robot_telemetry_map,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.robot_telemetry_scan,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.robot_telemetry_odom,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.robot_telemetry_imu,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.robot_telemetry_tf,
    g_amr_mqtt_bridge_config.mqtt.command_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.robot_telemetry_tf_static,
    g_amr_mqtt_bridge_config.mqtt.command_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.robot_telemetry_joint_states,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  mqtt_rc = MQTTClient_subscribe(
    g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_config.mqtt.robot_telemetry_robot_description,
    g_amr_mqtt_bridge_config.mqtt.telemetry_qos);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    return mqtt_rc;
  }

  g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = true;
  return MQTTCLIENT_SUCCESS;
}

int amr_mqtt_bridge_connect_mqtt(void)
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

void amr_mqtt_bridge_disconnect_mqtt(void)
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

bool amr_mqtt_bridge_ensure_connected(void)
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

bool amr_mqtt_bridge_publish_payload(
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

bool amr_mqtt_bridge_publish_binary_payload(
  const char * mqtt_topic,
  const void * payload,
  size_t payload_length,
  int qos)
{
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
