#include "baserosmodule.h"
#include "ros.h"
#include <controls/pid.h>
#include <messages.h>
#include <rosserial_msgs/Log.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

#ifndef ROSMOTORMODULE_H
#define ROSMOTORMODULE_H

namespace unav::modules {
class RosMotorModule : public BaseRosModule {
public:
  void initialize();

protected:
  void moduleThreadStart();
  unav::controls::PID pidControllers[MOTORS_COUNT];
};
} // namespace unav::modules
#endif