#include <modules/commmodule.h>
#include <modules/motorcontrollermodule.h>
#include <modules/motormanagermodule.h>
#include <modules/systemmodule.h>
#pragma once
namespace unav {
class Modules {
  Modules() = delete;

public:
  inline static unav::modules::CommModule *rosNodeModule;
  inline static unav::modules::MotorControllerModule *motorControllerModule;
  inline static unav::modules::MotorManagerModule *motorManagerModule;
  inline static unav::modules::SystemModule *systemModule;

private:
};
} // namespace unav