#include "basemodule.h"
#include <drivers/ina219.h>
#include <drivers/lm75.h>
#include <messages.h>
#include <utils/timer.h>

#pragma once

namespace unav::modules {
#define SYSTEMSTACKSIZE 1024
class SystemModule : public BaseModule<SYSTEMSTACKSIZE> {
public:
  static constexpr char *ModuleName = "SysMod";

  SystemModule();
  virtual void initialize();

protected:
  void moduleThreadStart() __attribute__((noreturn));
  void setup();
  unav::drivers::Lm75 thermometer;
  unav::drivers::Ina219 power;

private:
  unav::utils::Timer timer;
};
} // namespace unav::modules
