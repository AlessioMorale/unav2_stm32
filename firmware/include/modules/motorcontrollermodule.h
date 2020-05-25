#include <configuration.h>
#include <controls/pid.h>
#include <drivers/motors.h>
#include <main.h>
#include <messages.h>
#include <modules/baserosmodule.h>
#include <utils/timer.h>
#pragma once
#define ConversionPerChannel 1

namespace unav::modules {

class MotorControllerModule : protected BaseRosModule {
public:
  MotorControllerModule();
  void initialize();
  static const uint32_t ModuleMessageId{0x0A02};
  static constexpr char const *ModuleName = "MotCtl";

protected:
  virtual void moduleThreadStart() __attribute__((noreturn));

private:
  unav::drivers::Motors<MOTORS_COUNT> motors;
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
  void checkMessages(bool wait);
  void updatePidConfig(const pidconfig_content_t *cfg);
  void updateLimitsConfig(const limitsconfig_content_t *cfg);
  void updateSafetyConfig(const safetyconfig_content_t *cfg);
  void updateBridgeConfig(const bridgeconfig_content_t *cfg);
  void updateOperationConfig(const operationconfig_content_t *cfg);
  void updateTimings(const float frequency);
};
} // namespace unav::modules
