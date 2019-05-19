
#include "modules/rosnodemodule.h"
#include "FreeRTOS.h"
namespace unav::modules {

void RosNodeModule::initialize() {
  getNodeHandle().initNode();
  BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 512);
}
void RosNodeModule::moduleThreadStart() {
  TickType_t c = xTaskGetTickCount();
  while (true) {
    // getNodeHandle().loginfo(hello);
    getNodeHandle().spinOnce();
    vTaskDelayUntil(&c, 2);
  }
}
} // namespace unav::modules