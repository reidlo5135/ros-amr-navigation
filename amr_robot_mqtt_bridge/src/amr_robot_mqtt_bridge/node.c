#include "amr_robot_mqtt_bridge/node.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <rcutils/logging_macros.h>

rcl_allocator_t allocator_;
rclc_support_t support_;
rcl_node_t node_;
bool support_initialized_ = false;
bool node_initialized_ = false;

rcl_ret_t amr_robot_mqtt_bridge_initialize(int argc, const char *argv[])
{
  rcl_ret_t rc;

  allocator_ = rcl_get_default_allocator();
  (void)memset(&support_, 0, sizeof(support_));
  node_ = rcl_get_zero_initialized_node();

  rc = rclc_support_init(&support_, argc, argv, &allocator_);
  if (rc != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED("amr_robot_mqtt_bridge", "Failed to initialize rclc support");
    return rc;
  }
  support_initialized_ = true;

  rc = rclc_node_init_default(
    &node_,
    AMR_ROBOT_MQTT_BRIDGE_NODE_NAME,
    AMR_ROBOT_MQTT_BRIDGE_NODE_NAMESPACE,
    &support_);
  if (rc != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED("amr_robot_mqtt_bridge", "Failed to initialize node");
    return rc;
  }
  node_initialized_ = true;

  RCUTILS_LOG_INFO_NAMED(
    "amr_robot_mqtt_bridge",
    "Initialized amr_robot_mqtt_bridge scaffold for TurtleBot3 platform topics");
  return RCL_RET_OK;
}

void amr_robot_mqtt_bridge_run(void)
{
  RCUTILS_LOG_INFO_NAMED(
    "amr_robot_mqtt_bridge",
    "Waiting for ROS <-> MQTT edge bindings to be attached");

  while (rcl_context_is_valid(&support_.context)) {
    usleep(100000);
  }
}

rcl_ret_t amr_robot_mqtt_bridge_terminate(void)
{
  rcl_ret_t rc = RCL_RET_OK;

  if (node_initialized_) {
    rc = rcl_node_fini(&node_);
    if (rc != RCL_RET_OK) {
      return rc;
    }
    node_initialized_ = false;
  }

  if (support_initialized_) {
    rc = rclc_support_fini(&support_);
    if (rc != RCL_RET_OK) {
      return rc;
    }
    support_initialized_ = false;
  }

  return RCL_RET_OK;
}
