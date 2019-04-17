#include "modules/basemodule.h"
#include "cmsis_os.h"

namespace unav::modules{ 
void BaseModule::initialize(osPriority priority, int tasksize){
    osThreadDef(moduleThread, BaseModuleTaskWrapper, priority, 0, tasksize);
    moduleThread = osThreadCreate(osThread(moduleThread), (void*)this);
}

extern "C" void BaseModuleTaskWrapper(void const * argument){
    BaseModule* module = static_cast<BaseModule*> ((void*)argument);		
    module->moduleThreadStart();
}

}
