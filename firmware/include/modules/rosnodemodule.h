#include "baserosmodule.h"
#include "message_buffer.h"
#include "ros.h"
#include <configuration.h>
#include <counters.h>
#include <instrumentation/instrumentation.h>
#include <messages.h>
#include <modules/motormanagermodule.h>
#include <std_msgs/UInt32.h>
#include <unav2_msgs/BridgeConfig.h>
#include <unav2_msgs/Diagnostic.h>
#include <unav2_msgs/EncoderConfig.h>
#include <unav2_msgs/JointCommand.h>
#include <unav2_msgs/JointState.h>
#include <unav2_msgs/MechanicalConfig.h>
#include <unav2_msgs/OperationConfig.h>
#include <unav2_msgs/PIDConfig.h>
#include <unav2_msgs/PIDState.h>
#include <unav2_msgs/PerfCounter.h>
#include <unav2_msgs/SafetyConfig.h>
#include <unav2_msgs/SystemStatus.h>

#ifndef ROSNODEMODULE_H
#define ROSNODEMODULE_H

namespace unav::modules {
#define MESSAGING_BUFFER_SIZE 10
#define ROSNODESTACKSIZE 512

class RosNodeModule : public BaseRosModule<ROSNODESTACKSIZE> {
public:
  static const uint32_t ModuleMessageId = BaseRosModule::RosNodeModuleMessageId;
  static constexpr char *ModuleName = "ROSNode";

  RosNodeModule();
  void initialize();
  unav2_msgs::PerfCounter msgPerfCounter[COUNTERS_COUNT];

private:
  void sendRosMessage(message_t *msg);
  static ros::NodeHandle nh;
  ros::NodeHandle &getNodeHandle() {
    return nh;
  }

  QueueHandle_t incomingMessageQueue;
  message_t _messageBuffer[MESSAGING_BUFFER_SIZE];
  template <typename T> void sendToMotorManager(const T &msg);
  template <typename T> void handleRosConfigMessage(const T &msg);

  void publishDiagnostic();
  unav2_msgs::JointState msgjointstate;
  unav2_msgs::PIDState msgpidstate;
  unav2_msgs::Diagnostic msgdiagnostic;
  unav2_msgs::SystemStatus msgsystemstatus;

  std_msgs::UInt32 msgack;
  ros::Publisher pubJoints;
  ros::Publisher pubVelPIDState;
  ros::Publisher pubCurPIDState;
  ros::Publisher pubAck;
  ros::Publisher pubDiagnostic;
  ros::Publisher pubSystemStatus;
  ros::Subscriber<unav2_msgs::JointCommand> subJointCmd;
  ros::Subscriber<unav2_msgs::BridgeConfig> subBridgeCfg;
  ros::Subscriber<unav2_msgs::EncoderConfig> subEncoderCfg;
  ros::Subscriber<unav2_msgs::MechanicalConfig> subMechanicalCfg;
  ros::Subscriber<unav2_msgs::OperationConfig> subOperationCfg;
  ros::Subscriber<unav2_msgs::PIDConfig> subPIDCfg;
  ros::Subscriber<unav2_msgs::SafetyConfig> subSafetyCfg;

protected:
  void moduleThreadStart() __attribute__((noreturn));
};
} // namespace unav::modules
#endif
