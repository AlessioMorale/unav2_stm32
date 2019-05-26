#include "cmsis_os.h"
#ifndef BASEMODULE_H
#define BASEMODULE_H
namespace unav::modules {
extern "C" void BaseModuleTaskWrapper(void const *argument);
class BaseModule {
public:
  virtual void initialize(osPriority priority, int tasksize);

protected:
  osThreadId moduleThread;
  virtual void moduleThreadStart() __attribute__((noreturn));
  friend void BaseModuleTaskWrapper(void const *argument);
};
} // namespace unav::modules
#endif
