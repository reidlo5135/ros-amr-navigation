#ifndef AMR_ROBOT_MQTT_BRIDGE__NODE_H_
#define AMR_ROBOT_MQTT_BRIDGE__NODE_H_

#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>

#define AMR_ROBOT_MQTT_BRIDGE_NODE_NAME "robot_mqtt_bridge"
#define AMR_ROBOT_MQTT_BRIDGE_NODE_NAMESPACE "/amr"

extern rcl_allocator_t allocator_;
extern rclc_support_t support_;
extern rcl_node_t node_;
extern bool support_initialized_;
extern bool node_initialized_;

rcl_ret_t amr_robot_mqtt_bridge_initialize(int argc, const char *argv[]);
void amr_robot_mqtt_bridge_run(void);
rcl_ret_t amr_robot_mqtt_bridge_terminate(void);

#endif
