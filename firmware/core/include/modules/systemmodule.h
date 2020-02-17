#include "baserosmodule.h"
#include "ros.h"
#include <drivers/ina219.h>
#include <drivers/lm75.h>
#include <messages.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <utils/timer.h>

#pragma once

namespace unav::modules {
class SystemModule : public BaseRosModule {
public:
  static const uint32_t ModuleMessageId{0x0515};

  SystemModule();
  virtual void initialize();

protected:
  void moduleThreadStart() __attribute__((noreturn));
  void setup();
  unav::drivers::Lm75 thermometer;

private:
  unav::utils::Timer timer;
  void checkMessages();
};
} // namespace unav::modules
