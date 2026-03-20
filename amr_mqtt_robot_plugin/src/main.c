#include "amr_mqtt_robot_plugin/node.h"

int main(int argc, const char *argv[])
{
  rcl_ret_t rc = amr_mqtt_robot_plugin_initialize(argc, argv);
  if (rc != RCL_RET_OK) {
    return (int)rc;
  }

  amr_mqtt_robot_plugin_run();
  rc = amr_mqtt_robot_plugin_terminate();
  return (int)rc;
}
