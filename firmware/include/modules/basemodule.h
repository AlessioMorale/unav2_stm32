#include <FreeRTOS.h>
#ifndef BASEMODULE_H
#define BASEMODULE_H
namespace unav::modules {
extern "C" void BaseModuleTaskWrapper(void *argument);
class BaseRunnableModule {
  public:
    virtual void moduleThreadStart() __attribute__((noreturn)) = 0;
};

template <configSTACK_DEPTH_TYPE stackSize> class BaseModule : public BaseRunnableModule {
public:
  void initializeTask(osPriority priority, const char *const taskName) {
    xTaskCreate(BaseModuleTaskWrapper, taskName, stackSize, static_cast<void *>(this), priority, &moduleThread);
  }

  virtual ~BaseModule() = default;
protected:
  BaseModule() : moduleThread{nullptr} {};
  TaskHandle_t moduleThread;
  virtual void moduleThreadStart() __attribute__((noreturn)) = 0;
  friend void BaseModuleTaskWrapper(void const *argument);
};
} // namespace unav::modules
#endif
