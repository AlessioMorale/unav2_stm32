#include "baserosmodule.h"
#include "ros.h"
#include <controls/pid.h>
#include <drivers/encoder.h>
#include <messages.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <utils/timer.h>

#ifndef ROSMOTORMODULE_H
#define ROSMOTORMODULE_H

namespace unav::modules {
class MotorManagerModule : public BaseRosModule {
public:
  static const uint32_t ModuleMessageId{0x0A01};

  MotorManagerModule();
  virtual void initialize();

protected:
  void moduleThreadStart() __attribute__((noreturn));

private:
  unav::utils::Timer timer;
  unav::drivers::Encoder encoders[MOTORS_COUNT];
  unav::controls::PID pidControllers[MOTORS_COUNT];
  float inverted_rotation[MOTORS_COUNT];
  uint32_t wait;
  jointcommand_mode_t mode;
  float nominalDt;
  float dt;
  float filteredEffort[2];
  float cmd[MOTORS_COUNT];
  float encoder_ppr;
  uint8_t pid_publish_rate;
  bool pid_debug;
  motorcontrol_mode_t control_mode;
  int timeout_counter = 0;

  void disableDrivers();
  void enableDrivers();

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
#endif