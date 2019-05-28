
#include "modules/rosnodemodule.h"
#include "FreeRTOS.h"
#include "modules/rosmotormodule.h"
#include <message_buffer.h>
#include <messageconverter.h>
#include <messaging.h>
#include <ros.h>
#include <timing.h>

namespace unav::modules {
ros::NodeHandle RosNodeModule::nh;
RosNodeModule *rosNode;

RosNodeModule::RosNodeModule()
    : pubJoints("unav2/status/joint", &msgjointstate),
      pubPIDState("unav2/status/vel_pid", &msgpidstate),
      pubAck("unav2/status/ack", &msgack),
      subCommand(
          "unav2/control/joint_cmd",
          [](const unav2_msgs::JointCommand &msg) {
            if (rosNode) {
              message_t *m = rosNode->prepareMessage();
              unav::MessageConverter<unav2_msgs::JointCommand>::fromRosMsg(msg,
                                                                           m);
              rosNode->sendMessage(
                  m, unav::modules::RosMotorModule::ModuleMessageId);
            }
          }),
      subPID("unav2/config/pid", [](const unav2_msgs::PIDConfig &msg) {
        if (rosNode) {
          message_t *m = rosNode->prepareMessage();
          unav::MessageConverter<unav2_msgs::PIDConfig>::fromRosMsg(msg, m);
          rosNode->sendMessage(m,
                               unav::modules::RosMotorModule::ModuleMessageId);
        }
      }) {
  rosNode = this;
}

void RosNodeModule::initialize() {
  getNodeHandle().initNode();
  getMessaging().setup((uint8_t *)_messageBuffer, sizeof(message_t),
                       MESSAGING_BUFFER_SIZE);
  subscribe(RosNodeModule::ModuleMessageId);
  BaseRosModule::initialize(osPriority::osPriorityNormal, 512);
}

void RosNodeModule::moduleThreadStart() {
  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubPIDState);
  getNodeHandle().advertise(pubAck);

  getNodeHandle().subscribe(subCommand);
  getNodeHandle().subscribe(subPID);

  auto t = timing_getMs();

  while (true) {
    message_t *msg{nullptr};
    while (waitMessage(&msg, 2)) {
      sendRosMessage(msg);
      releaseMessage(msg);
      auto now = timing_getMs();
      if (now - t > 5) {
        t = now;
        getNodeHandle().spinOnce();
      }
    }
    getNodeHandle().spinOnce();
  }
}

void RosNodeModule::sendRosMessage(message_t *msg) {
  switch (msg->type) {
  case unav::message_types_t::outbound_JointState:
    msgjointstate.stamp = getNodeHandle().now();
    unav::MessageConverter<unav2_msgs::JointState>::toRosMsg(msg,
                                                             msgjointstate);
    pubJoints.publish(&msgjointstate);
    break;
  case unav::message_types_t::outbound_PIDState:
    unav::MessageConverter<unav2_msgs::PIDState>::toRosMsg(msg, msgpidstate);
    pubPIDState.publish(&msgpidstate);
    break;
  case unav::message_types_t::outboudn_ack:
    msgack.data = msg->ackcontent.transactionId;
    pubAck.publish(&msgack);
    break;
  default:
    getNodeHandle().logwarn("sendRosMessage: Invalid message");
    break;
  }
}

} // namespace unav::modules
