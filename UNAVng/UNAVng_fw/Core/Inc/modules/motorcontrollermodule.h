#include <modules/baserosmodule.h>
#include <main.h>
#include <messages.h>
#include <controls/pid.h>

#ifndef MOTORCONTROLLERMODULE_H
#define MOTORCONTROLLERMODULE_H
namespace unav::modules{ 
class MotorControllerModule : protected BaseRosModule{
    public:
        void initialize();
    protected:
        virtual void moduleThreadStart();
    private:
        void setup();
        unav::controls::PID pidControllers[MOTORS_COUNT];
        bool MotorEnabled = false;
        const uint32_t Channels[MOTORS_COUNT] = MOTOR_CUR_ADC2_ARRAY_OF_CHANNELS;
        const uint32_t AdcSamplingTime = ADC_SAMPLETIME_15CYCLES;
#define ConversionPerChannel  4
        volatile uint16_t adcConversionBuffer[MOTORS_COUNT * ConversionPerChannel] = {0};
};
}
#endif //MOTORCONTROLLERMODULE_H