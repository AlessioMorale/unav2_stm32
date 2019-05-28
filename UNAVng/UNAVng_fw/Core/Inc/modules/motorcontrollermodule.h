#include <controls/pid.h>
#include <main.h>
#include <messages.h>
#include <modules/baserosmodule.h>

#ifndef MOTORCONTROLLERMODULE_H
#define MOTORCONTROLLERMODULE_H
namespace unav::modules {
class MotorControllerModule : protected BaseRosModule {
public:
  void initialize();
  static const uint32_t ModuleMessageId{0x0A02};

protected:
  virtual void moduleThreadStart() __attribute__((noreturn));

private:
  void setup();
  unav::controls::PID pidControllers[MOTORS_COUNT];
  bool MotorEnabled = false;
  const uint32_t Channels[MOTORS_COUNT] = MOTOR_CUR_ADC2_ARRAY_OF_CHANNELS;
  const uint32_t AdcSamplingTime = ADC_SAMPLETIME_15CYCLES;
#define ConversionPerChannel 1
  volatile uint16_t adcConversionBuffer[MOTORS_COUNT * ConversionPerChannel] = {
      0};
};
} // namespace unav::modules
#endif // MOTORCONTROLLERMODULE_H