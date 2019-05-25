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
#define ACK_QUEUE_LENGTH 10
#define ACK_QUEUE_ITEM_SIZE                                                    \
  (sizeof(unav2_msgs::BridgeConfig::_transactionId_type))
class RosMotorModule : public BaseRosModule {
public:
  static const uint32_t ModuleMessageId = 0x0A01;
  RosMotorModule();
  void initialize();

protected:
  QueueHandle_t _incomingMessageQueue;
  unav::utils::Timer timer;
  unav::drivers::Encoder encoders[MOTORS_COUNT];
  void moduleThreadStart();
  unav::controls::PID pidControllers[MOTORS_COUNT];
};
} // namespace unav::modules
#endif