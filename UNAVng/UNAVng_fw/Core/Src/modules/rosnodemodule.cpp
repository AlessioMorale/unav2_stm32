
#include "modules/rosnodemodule.h"
#include "FreeRTOS.h"
#include "modules/motormanagermodule.h"
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
      subJointCmd("unav2/control/joint_cmd",
                  [](const unav2_msgs::JointCommand &msg) {
                    rosNode->handleRosMessage<unav2_msgs::JointCommand>(
                        msg, unav::modules::MotorManagerModule::ModuleMessageId);
                  }),
      subPIDCfg("unav2/config/pid",
                [](const unav2_msgs::PIDConfig &msg) {
                  rosNode->handleRosMessage<unav2_msgs::PIDConfig>(
                      msg, unav::modules::MotorManagerModule::ModuleMessageId);
                }),
      subBridgeCfg("unav2/config/bridge",
                   [](const unav2_msgs::BridgeConfig &msg) {
                     rosNode->handleRosMessage<unav2_msgs::BridgeConfig>(
                         msg, unav::modules::MotorManagerModule::ModuleMessageId);
                   }),
      subEncoderCfg("unav2/config/encoder",
                    [](const unav2_msgs::EncoderConfig &msg) {
                      rosNode->handleRosMessage<unav2_msgs::EncoderConfig>(
                          msg, unav::modules::MotorManagerModule::ModuleMessageId);
                    }),
      subLimitCfg("unav2/config/limits",
                  [](const unav2_msgs::LimitsConfig &msg) {
                    rosNode->handleRosMessage<unav2_msgs::LimitsConfig>(
                        msg, unav::modules::MotorManagerModule::ModuleMessageId);
                  }),
      subMechanicalCfg(
          "unav2/config/mechanical",
          [](const unav2_msgs::MechanicalConfig &msg) {
            rosNode->handleRosMessage<unav2_msgs::MechanicalConfig>(
                msg, unav::modules::MotorManagerModule::ModuleMessageId);
          }),
      subOperationCfg("unav2/config/operation",
                      [](const unav2_msgs::OperationConfig &msg) {
                        rosNode->handleRosMessage<unav2_msgs::OperationConfig>(
                            msg,
                            unav::modules::MotorManagerModule::ModuleMessageId);
                      }),
      subSafetyCfg("unav2/config/safety",
                   [](const unav2_msgs::SafetyConfig &msg) {
                     rosNode->handleRosMessage<unav2_msgs::SafetyConfig>(
                         msg, unav::modules::MotorManagerModule::ModuleMessageId);
                   })

{
  rosNode = this;
}

template <typename T>
void RosNodeModule::handleRosMessage(const T &msg, uint32_t destination) {
  message_t *m = rosNode->prepareMessage();
  unav::MessageConverter<T>::fromRosMsg(msg, m);
  rosNode->sendMessage(m, destination);
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

  getNodeHandle().subscribe(subJointCmd);
  getNodeHandle().subscribe(subBridgeCfg);
  getNodeHandle().subscribe(subEncoderCfg);
  getNodeHandle().subscribe(subLimitCfg);
  getNodeHandle().subscribe(subMechanicalCfg);
  getNodeHandle().subscribe(subOperationCfg);
  getNodeHandle().subscribe(subPIDCfg);
  getNodeHandle().subscribe(subSafetyCfg);

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
