#pragma once
#include <board.h>
#include <configuration.h>
#include <controls/pid.h>
#include <drivers/motor.h>
#include <messages.h>
#include <modules/baserosmodule.h>
#include <simplemessaging.h>
#include <utils/timer.h>

#define ConversionPerChannel 1

namespace unav::modules {
#define MOTORCONTROLLERSTACKSIZE 1024
#define MOTORCONTROLLERSTACKSIZE 1024

class MotorControllerModule : 
  protected BaseRosModule<MOTORCONTROLLERSTACKSIZE>,
  public ConfigurationObserver {
public:
  static constexpr char *ModuleName{"MotCtl"};
  MotorControllerModule();
  void initialize();

  void processMessage(internal_message_t &message);
  void configurationUpdated(const unav::ConfigurationMessageTypes_t configuredItem);
  motorcontrol_mode_t getStatus(){
    return mode;
  }

protected:
  virtual void moduleThreadStart() __attribute__((noreturn));

private:
  bool driversEnabled;
  unav::drivers::Motor motors[MOTORS_COUNT];
  int timeoutCounter;
  bool curLoopEnabled;
  float cmd[MOTORS_COUNT];
  unav::utils::Timer timer;
  motorcontrol_mode_t mode;
  uint8_t pid_publish_rate;
  bool pid_debug;
  float nominalDt;
  float dt;
  unav::controls::PID pidControllers[MOTORS_COUNT];
  unav::SimpleMessaging<internal_message_t, 5> internalMessaging;
  void setup();
  void checkMessages();
  void updatePidConfig();
  void updateLimitsConfig();
  void updateSafetyConfig();
  void updateBridgeConfig();
  void updateOperationConfig();
  void updateTimings(const float frequency);
};
} // namespace unav::modules