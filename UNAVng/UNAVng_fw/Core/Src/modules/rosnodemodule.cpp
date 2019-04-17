
#include "modules/rosnodemodule.h"
#include "FreeRTOS.h"
namespace unav::modules{

void RosNodeModule::initialize(){
    getNodeHandle().initNode();
    BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 512);
}
void RosNodeModule::moduleThreadStart(){
    TickType_t c = xTaskGetTickCount();
    const char hello[] = "hello world!\0";
    while (true) {
      //getNodeHandle().loginfo(hello);
      getNodeHandle().spinOnce();
      vTaskDelayUntil(&c, 10);  
    }
}
}