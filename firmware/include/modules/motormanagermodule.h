#include "baserosmodule.h"
#include "ros.h"
#include <configuration.h>
#include <controls/pid.h>
#include <drivers/encoder.h>
#include <messages.h>
#include <modules/motorcontrollermodule.h>
#include <simplemessaging.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <utils/timer.h>
#pragma once

namespace unav::modules {
#define MOTORMANAGERSTACKSIZE 1024
class MotorManagerModule : public BaseRosModule<MOTORMANAGERSTACKSIZE> {
public:
  static constexpr char *ModuleName{"MotMan"};

  MotorManagerModule();
  virtual void initialize();
  void processMessage(internal_message_t &message);

protected:
  void moduleThreadStart() __attribute__((noreturn));

private:
  int timeoutCounter;
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
  SimpleMessaging<internal_message_t, 5> internalMessaging;
  void configure(const reconfigure_content_t *reconfig);

  void updatePidConfig();
  void updateEncoderConfig();
  void updateBridgeConfig();
  void updateLimitsConfig();
  void updateMechanicalConfig();
  void updateOperationConfig();
  void updateSafetyConfig();
  void updateTimings(const float frequency);
  void checkMessages();
};
} // namespace unav::modules
