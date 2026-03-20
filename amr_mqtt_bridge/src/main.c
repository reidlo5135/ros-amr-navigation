#include "amr_mqtt_bridge/mqtt_bridge.h"

int main(int argc, const char * const * argv)
{
  int return_code = amr_mqtt_bridge_init(argc, argv);
  if (return_code != 0) {
    return return_code;
  }

  amr_mqtt_bridge_spin();
  return amr_mqtt_bridge_shutdown();
}
