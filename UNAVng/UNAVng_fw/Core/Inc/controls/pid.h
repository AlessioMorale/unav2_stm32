/*
** unav2
** File description:
** pid
*/

#pragma once
#include <control_msgs/PidState.h>
namespace unav::controls{
class PID{
    public:
        void setGains(float kp, float ki, float kd, float ilimit);
        float apply(float setpoint, float measure, float dT, control_msgs::PidState *state);
        void zero();
        void setRange(float min, float max);
    private:
        float _kp;
        float _ki;
        float _kd;
        float _iLimit;
        float _iAccumulator;
        float _lastError;
        float _min;
        float _max;
};
}
