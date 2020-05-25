#include "baserosmodule.h"
#include "ros.h"
#include <configuration.h>
#include <controls/pid.h>
#include <drivers/encoder.h>
#include <messages.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <utils/timer.h>
#pragma once

namespace unav::modules {
class MotorManagerModule : public BaseRosModule {
public:
  static const uint32_t ModuleMessageId{0x0A01};
  static constexpr char *ModuleName{"MotMan"};

  MotorManagerModule();
  virtual void initialize();

protected:
  void moduleThreadStart() __attribute__((noreturn));

private:
  unav::utils::Timer timer;
  unav::drivers::Encoder encoders[MOTORS_COUNT];
  float filteredEffort[2];
  jointcommand_mode_t mode;
  unav::controls::PID pidControllers[MOTORS_COUNT];
  uint32_t wait;
  float nominalDt;
  float dt;
  float cmd[MOTORS_COUNT];
  uint8_t pid_publish_rate;
  bool pid_debug;
  motorcontrol_mode_t control_mode;
  float inverted_rotation[MOTORS_COUNT];

  void updateConfiguration(const reconfigure_content_t *reconfig);
  void updatePidConfig(const pidconfig_content_t *cfg);
  void updateEncoderConfig(const encoderconfig_content_t *cfg);
  void updateBridgeConfig(const bridgeconfig_content_t *cfg);
  void updateLimitsConfig(const limitsconfig_content_t *cfg);
  void updateMechanicalConfig(const mechanicalconfig_content_t *cfg);
  void updateOperationConfig(const operationconfig_content_t *cfg);
  void updateSafetyConfig(const safetyconfig_content_t *cfg);
  void updateTimings(const float frequency);
  void checkMessages();
};
} // namespace unav::modules
