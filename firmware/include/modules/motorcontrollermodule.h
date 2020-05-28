#include <atomic>
#include <configuration.h>
#include <controls/pid.h>
#include <drivers/motor.h>
#include <board.h>
#include <messages.h>
#include <modules/baserosmodule.h>
#include <utils/timer.h>
#pragma once
#define ConversionPerChannel 1

namespace unav::modules {
#define MOTORCONTROLLERSTACKSIZE 1024
#define MOTORCONTROLLERSTACKSIZE 1024

class MotorControllerModule : protected BaseRosModule<MOTORCONTROLLERSTACKSIZE> {
public:
  static constexpr char *ModuleName{"MotCtl"};
  MotorControllerModule();
  void initialize();
  void setCommand(motorcontrol_content_t command);
  void configure(configuration_item_t configuration);

protected:
  virtual void moduleThreadStart() __attribute__((noreturn));

private:
  std::atomic_bool commandUpdated;
  std::atomic_bool configUpdated;
  uint32_t itemsToConfigure;
  unav::drivers::Motor motors[MOTORS_COUNT];
  int timeoutCounter = 0;
  bool curLoopEnabled;
  float cmd[MOTORS_COUNT];
  unav::utils::Timer timer;
  motorcontrol_mode_t mode;
  uint8_t pid_publish_rate;
  bool pid_debug;
  float nominalDt;
  float dt;
  unav::controls::PID pidControllers[MOTORS_COUNT];
  bool drivers_enabled = false;

  void setup();
  void updateConfiguration(const reconfigure_content_t *reconfig);
  void checkMessages();
  void updatePidConfig(const pidconfig_content_t *cfg);
  void updateLimitsConfig(const limitsconfig_content_t *cfg);
  void updateSafetyConfig(const safetyconfig_content_t *cfg);
  void updateBridgeConfig(const bridgeconfig_content_t *cfg);
  void updateOperationConfig(const operationconfig_content_t *cfg);
  void updateTimings(const float frequency);
};


} // namespace unav::modules
