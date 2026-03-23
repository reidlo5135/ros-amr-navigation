#ifndef AMR_MQTT_BRIDGE__MQTT_H_
#define AMR_MQTT_BRIDGE__MQTT_H_

#include <stdbool.h>
#include <stddef.h>

int amr_mqtt_bridge_connect_mqtt(void);
void amr_mqtt_bridge_disconnect_mqtt(void);
bool amr_mqtt_bridge_ensure_connected(void);
bool amr_mqtt_bridge_publish_payload(
  const char * mqtt_topic,
  const char * payload,
  int qos);
bool amr_mqtt_bridge_publish_binary_payload(
  const char * mqtt_topic,
  const void * payload,
  size_t payload_length,
  int qos);

#endif  // AMR_MQTT_BRIDGE__MQTT_H_
