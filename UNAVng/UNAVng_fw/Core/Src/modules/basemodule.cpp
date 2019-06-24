#include "modules/basemodule.h"
#include "cmsis_os.h"

namespace unav::modules {
void BaseModule::initializeTask(osPriority priority, int tasksize) {
  osThreadDef(moduleThread, BaseModuleTaskWrapper, priority, 0, tasksize);
  moduleThread =
      osThreadCreate(osThread(moduleThread), static_cast<void *>(this));
}

extern "C" void BaseModuleTaskWrapper(void const *argument) {
  BaseModule *module = static_cast<BaseModule *>(const_cast<void *>(argument));
  module->moduleThreadStart();
}

} // namespace unav::modules
