#include "amr_mqtt_server/node.h"

int main(int argc, const char *argv[])
{
  rcl_ret_t rc = amr_mqtt_bridge_initialize(argc, argv);
  if (rc != RCL_RET_OK)
  {
    return (int)rc;
  }

  amr_mqtt_bridge_run();
  rc = amr_mqtt_bridge_terminate();
  return (int)rc;
}
