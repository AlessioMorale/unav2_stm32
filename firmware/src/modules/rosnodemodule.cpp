
#include "FreeRTOS.h"
#include "modules/motormanagermodule.h"
#include <configurationmessageconverter.h>
#include <counters.h>
#include <leds.h>
#include <message_buffer.h>
#include <messageconverter.h>
#include <messaging.h>
#include <modules.h>
#include <modules/rosnodemodule.h>
#include <ros.h>
#include <timing.h>

namespace unav::modules {
ros::NodeHandle RosNodeModule::nh;
RosNodeModule *rosNode;

RosNodeModule::RosNodeModule()
    : incomingMessageQueue{nullptr}, pubJoints("unav2/status/joint", &msgjointstate), pubVelPIDState("unav2/status/vel_pid", &msgpidstate),
      pubCurPIDState("unav2/status/cur_pid", &msgpidstate), pubAck("unav2/status/ack", &msgack), pubDiagnostic("unav2/diagnostic", &msgDiagnostic),
      subJointCmd("unav2/control/joint_cmd", [](const unav2_msgs::JointCommand &msg) { rosNode->sendToMotorManager(msg); }),
      subPIDCfg("unav2/config/pid", [](const unav2_msgs::PIDConfig &msg) { rosNode->handleRosConfigMessage(msg); }),
      subBridgeCfg("unav2/config/bridge", [](const unav2_msgs::BridgeConfig &msg) { rosNode->handleRosConfigMessage(msg); }),
      subEncoderCfg("unav2/config/encoder", [](const unav2_msgs::EncoderConfig &msg) { rosNode->handleRosConfigMessage(msg); }),
      subMechanicalCfg("unav2/config/mechanical", [](const unav2_msgs::MechanicalConfig &msg) { rosNode->handleRosConfigMessage(msg); }),
      subOperationCfg("unav2/config/operation", [](const unav2_msgs::OperationConfig &msg) { rosNode->handleRosConfigMessage(msg); }),
      subSafetyCfg("unav2/config/safety", [](const unav2_msgs::SafetyConfig &msg) { rosNode->handleRosConfigMessage(msg); }) {
  rosNode = this;
}

template <typename T> void RosNodeModule::sendToMotorManager(const T &msg) {
  internal_message_t m;
  unav::MessageConverter<T>::fromRosMsg(msg, m);
  unav::Modules::motorManagerModule->processMessage(m);
}

template <typename T> void RosNodeModule::handleRosConfigMessage(const T &msg) {
  static configuration_message_t c;
  unav::ConfigurationMessageConverter<T>::fromRosMsg(msg, &c);
  Application::configuration.set(&c);
  msgack.data = c.header.transactionId;
  pubAck.publish(&msgack);
}

void RosNodeModule::initialize() {
  instrumentation_init(COUNTERS_COUNT);
  initCounters();

  getNodeHandle().initNode();
  Application::messaging.setup((uint8_t *)_messageBuffer, sizeof(message_t), MESSAGING_BUFFER_SIZE);
  subscribe(RosNodeModule::ModuleMessageId, RosNodeModule::ModuleName);
  initializeTask(osPriority::osPriorityNormal, RosNodeModule::ModuleName);
}

void RosNodeModule::moduleThreadStart() {
  getNodeHandle().advertise(pubDiagnostic);
  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubCurPIDState);
  getNodeHandle().advertise(pubVelPIDState);
  getNodeHandle().advertise(pubAck);

  getNodeHandle().subscribe(subJointCmd);
  getNodeHandle().subscribe(subBridgeCfg);
  getNodeHandle().subscribe(subEncoderCfg);
  getNodeHandle().subscribe(subMechanicalCfg);
  getNodeHandle().subscribe(subOperationCfg);
  getNodeHandle().subscribe(subPIDCfg);
  getNodeHandle().subscribe(subSafetyCfg);

  auto t = timing_getRaw();
  auto t_diag = timing_getRaw();
  auto t_request_config = timing_getRaw();
  bool configuration_complete = false;
  bool connected = true;

  while (true) {
    if (connected != getNodeHandle().connected()) {
      connected = !connected;
      if (connected) {
        leds_setPattern(0, &leds_pattern_doublefast);
      } else {
        leds_setPattern(0, &leds_pattern_slow);
      }
    }

    message_t *msg{nullptr};
    while (waitMessage(&msg, 2)) {
      sendRosMessage(msg);
      releaseMessage(msg);

      if (timing_getUsSince(t) > 5000ul) {
        t = timing_getRaw();
        getNodeHandle().spinOnce();
      }
    }

    if (!configuration_complete && timing_getUsSince(t_request_config) > 250'000ul) {
      t_request_config = timing_getRaw();
      if(!Application::healthChecker.isConfigured()){
        msgack.data = MESSAGE_ACK_REQUEST_CONFIG;
        pubAck.publish(&msgack);
      } else {
        configuration_complete = true;
      }
    }

    if (timing_getUsSince(t_diag) > 100000ul) {
      t_diag = timing_getRaw();
      publishDiagnostic();
    }
    getNodeHandle().spinOnce();
  }
}

void RosNodeModule::publishDiagnostic() {
  instrumentation_forEachCounter([](const perf_counter_t *counter, const int8_t index,
                                    void *context) { MessageConverter<unav2_msgs::PerfCounter>::toRosMsg(counter, rosNode->msgPerfCounter[index]); },
                                 nullptr);
  msgDiagnostic.counters = msgPerfCounter;
  msgDiagnostic.counters_length = COUNTERS_COUNT;
  pubDiagnostic.publish(&msgDiagnostic);
}

void RosNodeModule::sendRosMessage(message_t *msg) {
  switch (msg->type) {
  case unav::message_types_t::outbound_JointState:
    unav::MessageConverter<unav2_msgs::JointState>::toRosMsg(msg, msgjointstate);
    msgjointstate.stamp = getNodeHandle().now();
    pubJoints.publish(&msgjointstate);
    break;
  case unav::message_types_t::outbound_VelPIDState:
    unav::MessageConverter<unav2_msgs::PIDState>::toRosMsg(msg, msgpidstate);
    msgpidstate.stamp = getNodeHandle().now();
    pubVelPIDState.publish(&msgpidstate);
    break;
  case unav::message_types_t::outbound_CurPIDState:
    unav::MessageConverter<unav2_msgs::PIDState>::toRosMsg(msg, msgpidstate);
    msgpidstate.stamp = getNodeHandle().now();
    pubCurPIDState.publish(&msgpidstate);
    break;
  case unav::message_types_t::outbound_ack:
    msgack.data = msg->ackcontent.transactionId;
    pubAck.publish(&msgack);
    break;
  default:
    getNodeHandle().logwarn("sendRosMessage: Invalid message");
    break;
  }
}

template class BaseRosModule<ROSNODESTACKSIZE>;

} // namespace unav::modules
