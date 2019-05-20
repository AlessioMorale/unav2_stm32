#include "baserosmodule.h"
#include "ros.h"
#include <controls/pid.h>
#include <messages.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <utils/timer.h>
#include <drivers/encoder.h>

#ifndef ROSMOTORMODULE_H
#define ROSMOTORMODULE_H

namespace unav::modules {
#define ACK_QUEUE_LENGTH 10
#define ACK_QUEUE_ITEM_SIZE (sizeof(unav2_msgs::BridgeConfig::_transactionId_type))
class RosMotorModule : public BaseRosModule {
public:
  RosMotorModule();
  void initialize();

protected:
  unav::utils::Timer timer;
  unav::drivers::Encoder encoders[MOTORS_COUNT];
  jointstate_content_t jointstate;
  pidstate_content_t pidstate;
  void moduleThreadStart();
  unav::controls::PID pidControllers[MOTORS_COUNT];
};
} // namespace unav::modules
#endif