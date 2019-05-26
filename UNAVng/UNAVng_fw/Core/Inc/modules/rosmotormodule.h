#include "baserosmodule.h"
#include "ros.h"
#include <controls/pid.h>
#include <drivers/encoder.h>
#include <messages.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <utils/timer.h>

#ifndef ROSMOTORMODULE_H
#define ROSMOTORMODULE_H

namespace unav::modules {
class RosMotorModule : public BaseRosModule {
public:
  static const uint32_t ModuleMessageId{0x0A01};
  RosMotorModule();
  virtual void initialize();

protected:
  unav::utils::Timer timer;
  unav::drivers::Encoder encoders[MOTORS_COUNT];
  void moduleThreadStart() __attribute__ ((noreturn));
  unav::controls::PID pidControllers[MOTORS_COUNT];
};
} // namespace unav::modules
#endif