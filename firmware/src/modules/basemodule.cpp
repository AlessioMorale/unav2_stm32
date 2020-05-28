#include "modules/basemodule.h"
#include <FreeRTOS.h>

namespace unav::modules {

extern "C" void BaseModuleTaskWrapper( void *argument) {
  BaseRunnableModule *module = static_cast<BaseRunnableModule *>(const_cast<void *>(argument));
  module->moduleThreadStart();
}

} // namespace unav::modules
