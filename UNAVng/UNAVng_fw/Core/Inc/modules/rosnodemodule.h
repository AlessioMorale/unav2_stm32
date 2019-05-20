#include "baserosmodule.h"
#include "ros.h"
#include "message_buffer.h"
#include <unav2_msgs/JointState.h>
#include <unav2_msgs/PIDState.h>
#include <std_msgs/UInt32.h>
#include <messages.h>
#ifndef ROSNODEMODULE_H
#define ROSNODEMODULE_H

namespace unav::modules {
#define OUTBOUND_BUFFER_SIZE 1000

class RosNodeModule : public BaseRosModule {
public:
  RosNodeModule();
  void initialize();
private:
  void setupMessages();
  void sendRosMessage();
  static ros::NodeHandle nh;
  ros::NodeHandle &getNodeHandle() { return nh; }
  uint8_t outboundBuffer[ OUTBOUND_BUFFER_SIZE ];
  StaticMessageBuffer_t outboundBufferStruct;
  MessageBufferHandle_t outboundMessageBuffer;
  outbound_message_t outboundMessage;
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