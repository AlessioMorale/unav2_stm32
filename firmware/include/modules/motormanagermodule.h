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
class MotorManagerModule : public BaseRosModule<MOTORMANAGERSTACKSIZE>, public ConfigurationObserver {
public:
  enum class status_t : int32_t { failsafe = -2, unconfigured = -1, stopped = 0, starting = 1, running = 2, stopping = 3 };

  static constexpr char *ModuleName{"MotMan"};

  MotorManagerModule();
  virtual void initialize();
  void processMessage(internal_message_t &message);
  void configurationUpdated(const unav::ConfigurationMessageTypes_t configuredItem);
  
  status_t getStatus(){
    return status;
  };

protected:
  void moduleThreadStart() __attribute__((noreturn));

private:
  const float ENC_CUTOFF{0.0f};
  const float CONNECTION_TIMEOUT{0.250f};
  const float ZERO_TIMEOUT{5.0f};


  bool operationModeNormal;
  status_t status;
  utils::Timer connectionTimeout;
  utils::Timer zeroTimeout;
  utils::Timer errorLimitTimeout;
  float errorlimit{0.0f};
  float errorConditionTimeout{0.0f};

  unav::utils::Timer timer;
  unav::drivers::Encoder encoders[MOTORS_COUNT];
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
  float loopFrequency;
  bool frequencyUpdated{false};

  void stop();
  void runControlLoop();
  void checkMessages();
  void updatePidConfig();
  void updateEncoderConfig();
  void updateBridgeConfig();
  void updateMechanicalConfig();
  void updateOperationConfig();
  void updateSafetyConfig();
  void updateTimings(const float frequency);
};
} // namespace unav::modules
