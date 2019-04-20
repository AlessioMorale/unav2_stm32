#include<controls/pid.h>
#include<mathutils.h>
namespace unav::controls
{
void PID::setGains(float kp, float ki, float kd, float ilimit){
    _kp = kp;
    _kd = kd;
    _ki = ki;
    _iLimit = ilimit;
}

void PID::setRange(float min, float max){
    _min = min;
    _max = max;
}

void PID::zero(){
    _lastError = 0;
    _iAccumulator = 0;
}

float PID::apply(float setpoint, float measure, float dT, control_msgs::PidState *state){

    float error = setpoint - measure;
    state->error = error;
    _iAccumulator += error * _ki * dT;
    _iAccumulator = fboundf( -_iLimit, _iLimit, _iAccumulator);

    float diff;

    diff = error - _lastError;
    state->d_error = diff;
    _lastError = error;

    float dterm = 0;

    if (_kd > 0.0f && dT > 0.0f) {
        dterm = diff * _kd / dT;
    }
    state->d_term = dterm;
    state->i_term = _iAccumulator;
    float pterm = (error * _kp);
    state->p_error = error;
    state->p_term = pterm;
    float output =  pterm + _iAccumulator + dterm;
    state->output = output;
    output = fboundf(_min, _max, output);
    return output;
}
} // namespace unav::controls