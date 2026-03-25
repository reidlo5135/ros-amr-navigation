#include "amr_mqtt_bridge/mqtt.h"

static int amr_mqtt_bridge_subscribe_command_topics(void)
{
  const char * command_topics[] = {
    g_amr_mqtt_bridge_config.mqtt.command_cmd_vel,
    g_amr_mqtt_bridge_config.mqtt.command_set_initial_pose,
    g_amr_mqtt_bridge_config.mqtt.command_navigate_to_pose,
    g_amr_mqtt_bridge_config.mqtt.command_cancel_navigate_to_pose,
    g_amr_mqtt_bridge_config.mqtt.command_ping
  };
  const char * request_topics[] = {
    g_amr_mqtt_bridge_config.mqtt.request_plan_segment,
    g_amr_mqtt_bridge_config.mqtt.request_plan_route
  };

  for (size_t index = 0; index < (sizeof(command_topics) / sizeof(command_topics[0])); ++index) {
    int mqtt_rc = MQTTClient_subscribe(
      g_amr_mqtt_bridge_mqtt.client,
      command_topics[index],
      g_amr_mqtt_bridge_config.mqtt.command_qos);
    if (mqtt_rc != MQTTCLIENT_SUCCESS) {
      RCUTILS_LOG_ERROR_NAMED(
        "amr_mqtt_bridge",
        "Failed to subscribe MQTT command topic '%s': rc=%d (%s)",
        command_topics[index],
        mqtt_rc,
        MQTTClient_strerror(mqtt_rc));
      return mqtt_rc;
    }
  }

  for (size_t index = 0; index < (sizeof(request_topics) / sizeof(request_topics[0])); ++index) {
    int mqtt_rc = MQTTClient_subscribe(
      g_amr_mqtt_bridge_mqtt.client,
      request_topics[index],
      g_amr_mqtt_bridge_config.mqtt.service_qos);
    if (mqtt_rc != MQTTCLIENT_SUCCESS) {
      RCUTILS_LOG_ERROR_NAMED(
        "amr_mqtt_bridge",
        "Failed to subscribe MQTT request topic '%s': rc=%d (%s)",
        request_topics[index],
        mqtt_rc,
        MQTTClient_strerror(mqtt_rc));
      return mqtt_rc;
    }
  }

  g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = true;
  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_bridge",
    "Subscribed MQTT command/request topics for cmd_vel, initial pose, navigation, cancel, and planner services");
  return MQTTCLIENT_SUCCESS;
}

int amr_mqtt_bridge_connect_mqtt(void)
{
  int mqtt_rc = 0;
  int broker_uri_length = snprintf(
    g_amr_mqtt_bridge_mqtt.broker_uri,
    sizeof(g_amr_mqtt_bridge_mqtt.broker_uri),
    "tcp://%s:%d",
    g_amr_mqtt_bridge_config.broker.host,
    g_amr_mqtt_bridge_config.broker.port);
  MQTTClient_connectOptions connect_options = MQTTClient_connectOptions_initializer;

  if (broker_uri_length < 0 ||
    (size_t)broker_uri_length >= sizeof(g_amr_mqtt_bridge_mqtt.broker_uri))
  {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "MQTT broker URI is too long for host '%s' and port %d",
      g_amr_mqtt_bridge_config.broker.host,
      g_amr_mqtt_bridge_config.broker.port);
    return 1;
  }

  RCUTILS_LOG_INFO_NAMED(
    "amr_mqtt_bridge",
    "Connecting to MQTT broker at %s",
    g_amr_mqtt_bridge_mqtt.broker_uri);

  mqtt_rc = MQTTClient_create(
    &g_amr_mqtt_bridge_mqtt.client,
    g_amr_mqtt_bridge_mqtt.broker_uri,
    g_amr_mqtt_bridge_config.broker.client_id,
    MQTTCLIENT_PERSISTENCE_NONE,
    NULL);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      "amr_mqtt_bridge",
      "Failed to create MQTT client for %s: rc=%d (%s)",
      g_amr_mqtt_bridge_mqtt.broker_uri,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
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
      "Failed to connect MQTT broker %s: rc=%d (%s)",
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
  g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = false;

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
      "Failed to reconnect MQTT broker %s: rc=%d (%s)",
      g_amr_mqtt_bridge_mqtt.broker_uri,
      mqtt_rc,
      MQTTClient_strerror(mqtt_rc));
    return false;
  }

  g_amr_mqtt_bridge_mqtt.connected = true;
  mqtt_rc = amr_mqtt_bridge_subscribe_command_topics();
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    g_amr_mqtt_bridge_mqtt.connected = false;
    g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = false;
    return false;
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
  int qos,
  bool retained)
{
  MQTTClient_message message = MQTTClient_message_initializer;
  MQTTClient_deliveryToken token = 0;
  int mqtt_rc = 0;

  if (!amr_mqtt_bridge_ensure_connected()) {
    return false;
  }

  message.payload = (void *)payload;
  message.payloadlen = (int)strlen(payload);
  message.qos = qos;
  message.retained = retained ? 1 : 0;

  mqtt_rc = MQTTClient_publishMessage(
    g_amr_mqtt_bridge_mqtt.client,
    mqtt_topic,
    &message,
    &token);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    if (mqtt_rc == MQTTCLIENT_DISCONNECTED) {
      g_amr_mqtt_bridge_mqtt.connected = false;
      g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = false;
    }
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
  int qos,
  bool retained)
{
  MQTTClient_message message = MQTTClient_message_initializer;
  MQTTClient_deliveryToken token = 0;
  int mqtt_rc = 0;

  if (!amr_mqtt_bridge_ensure_connected()) {
    return false;
  }

  message.payload = (void *)payload;
  message.payloadlen = (int)payload_length;
  message.qos = qos;
  message.retained = retained ? 1 : 0;

  mqtt_rc = MQTTClient_publishMessage(
    g_amr_mqtt_bridge_mqtt.client,
    mqtt_topic,
    &message,
    &token);
  if (mqtt_rc != MQTTCLIENT_SUCCESS) {
    if (mqtt_rc == MQTTCLIENT_DISCONNECTED) {
      g_amr_mqtt_bridge_mqtt.connected = false;
      g_amr_mqtt_bridge_mqtt.command_subscriptions_registered = false;
    }
    return false;
  }

  if (qos > 0) {
    (void)MQTTClient_waitForCompletion(g_amr_mqtt_bridge_mqtt.client, token, 1000L);
  }
  return true;
}
