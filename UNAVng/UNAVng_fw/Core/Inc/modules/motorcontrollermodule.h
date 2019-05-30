#include <controls/pid.h>
#include <main.h>
#include <messages.h>
#include <modules/baserosmodule.h>

#ifndef MOTORCONTROLLERMODULE_H
#define MOTORCONTROLLERMODULE_H
namespace unav::modules {
class MotorControllerModule : protected BaseRosModule {
public:
  MotorControllerModule();
  void initialize();
  static const uint32_t ModuleMessageId{0x0A02};
  static const uint32_t ModulePriorityMessageId{0x0A03};

protected:
  virtual void moduleThreadStart() __attribute__((noreturn));

private:
  void setup();
  bool waitPriorityMessage(message_t **msg, TickType_t wait) {
    return (xQueueReceive(_incomingPriorityMessageQueue, (void *)msg, wait) ==
            pdTRUE);
  }
  void checkMessages();
  void updatePidConfig(const pidconfig_content_t *cfg);
  void updateLimitsConfig(const limitsconfig_content_t *cfg);
  void updateSafetyConfig(const safetyconfig_content_t *cfg);
  void updateBridgeConfig(const bridgeconfig_content_t *cfg);
  void updateTimings(const float frequency);
  bool curLoopEnabled;
  QueueHandle_t _incomingPriorityMessageQueue;
  float cmd[MOTORS_COUNT];
  motorcontrol_mode_t mode;
  unav::controls::PID pidControllers[MOTORS_COUNT];
  bool MotorEnabled;
  const uint32_t Channels[MOTORS_COUNT] = MOTOR_CUR_ADC2_ARRAY_OF_CHANNELS;
  const uint32_t AdcSamplingTime = ADC_SAMPLETIME_15CYCLES;
#define ConversionPerChannel 1
  volatile uint16_t adcConversionBuffer[MOTORS_COUNT * ConversionPerChannel];
};
} // namespace unav::modules
#endif // MOTORCONTROLLERMODULE_H
