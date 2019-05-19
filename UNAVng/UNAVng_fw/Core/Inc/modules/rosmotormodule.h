#include "baserosmodule.h"
#include "ros.h"
#include <controls/pid.h>
#include <messages.h>
#include <rosserial_msgs/Log.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <unav2_msgs/BridgeConfig.h>
#ifndef ROSMOTORMODULE_H
#define ROSMOTORMODULE_H

namespace unav::modules {
#define ACK_QUEUE_LENGTH 10
#define ACK_QUEUE_ITEM_SIZE (sizeof(unav2_msgs::BridgeConfig::_transactionId_type))
class RosMotorModule : public BaseRosModule {
public:
  void initialize();

protected:
  void moduleThreadStart();
  StaticQueue_t _ackStaticQueue;
  uint8_t _ackQueueStorageArea[ ACK_QUEUE_LENGTH * ACK_QUEUE_ITEM_SIZE ];
  QueueHandle_t _ackQueue;
  unav::controls::PID pidControllers[MOTORS_COUNT];
};
} // namespace unav::modules
#endif