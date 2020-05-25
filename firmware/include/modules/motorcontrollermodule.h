#include <controls/pid.h>
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
  void disableDrivers();
  void enableDrivers();

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
  const uint32_t Channels[MOTORS_COUNT + 1] = MOTOR_CUR_ADC3_ARRAY_OF_CHANNELS;
  const uint32_t AdcSamplingTime = ADC_SAMPLETIME_15CYCLES;
  volatile uint16_t adcConversionBuffer[MOTORS_COUNT * ConversionPerChannel];

  void setup();
  void checkMessages(bool wait);
  void updatePidConfig(const pidconfig_content_t *cfg);
  void updateLimitsConfig(const limitsconfig_content_t *cfg);
  void updateSafetyConfig(const safetyconfig_content_t *cfg);
  void updateBridgeConfig(const bridgeconfig_content_t *cfg);
  void updateOperationConfig(const operationconfig_content_t *cfg);
  void updateTimings(const float frequency);
};
} // namespace unav::modules
