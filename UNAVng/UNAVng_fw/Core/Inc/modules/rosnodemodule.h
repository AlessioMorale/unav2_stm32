#include "baserosmodule.h"
#include "message_buffer.h"
#include "ros.h"
#include <messages.h>
#include <std_msgs/UInt32.h>
#include <unav2_msgs/JointState.h>
#include <unav2_msgs/PIDState.h>
#ifndef ROSNODEMODULE_H
#define ROSNODEMODULE_H

namespace unav::modules {
#define MESSAGING_BUFFER_SIZE 10

class RosNodeModule : public BaseRosModule {
public:
  static const uint32_t ModuleMessageId = 0x0100;
  RosNodeModule();
  void initialize();

private:
  void sendRosMessage(message_handle_t msg);
  static ros::NodeHandle nh;
  ros::NodeHandle &getNodeHandle() { return nh; }
  QueueHandle_t incomingMessageQueue;
  outbound_message_t _messageBuffer[MESSAGING_BUFFER_SIZE];

protected:
  unav2_msgs::JointState msgjointstate;
  unav2_msgs::PIDState msgpidstate;
  std_msgs::UInt32 msgack;
  ros::Publisher pubJoints;
  ros::Publisher pubPIDState;
  ros::Publisher pubAck;
  void moduleThreadStart();
};
} // namespace unav::modules
#endif