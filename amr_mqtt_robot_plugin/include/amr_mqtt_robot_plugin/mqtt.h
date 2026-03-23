#ifndef AMR_MQTT_ROBOT_PLUGIN__MQTT_H_
#define AMR_MQTT_ROBOT_PLUGIN__MQTT_H_

#include <stdbool.h>
#include <stddef.h>

int amr_mqtt_robot_plugin_connect_mqtt(void);
void amr_mqtt_robot_plugin_disconnect_mqtt(void);
bool amr_mqtt_robot_plugin_ensure_connected(void);
bool amr_mqtt_robot_plugin_publish_payload(
  const char * mqtt_topic,
  const char * payload,
  int qos,
  bool retained);
bool amr_mqtt_robot_plugin_publish_binary_payload(
  const char * mqtt_topic,
  const void * payload,
  size_t payload_length,
  int qos,
  bool retained);

#endif  // AMR_MQTT_ROBOT_PLUGIN__MQTT_H_
