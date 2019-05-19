#include<controls/pid.h>
#include<mathutils.h>
#include<math.h>
#include<limits.h>
#include<string.h>
namespace unav::controls
{
      PID::PID(){
        memset( &_status, 0, sizeof(pid_status_t));
        _mutex = xSemaphoreCreateBinaryStatic(&_semBuffer);
        release();
      }

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
    lock();
    _status.i_term = 0;
    _status.i_max = 0;
    _status.i_min = 0;
    release();
}

void PID::getStatus(pid_status_t *status){
    lock();
    memcpy(status, &_status, sizeof(pid_status_t));
    release();
}

float PID::apply(float setpoint, float measure, float dT){
    lock();
    _status.timestep = dT;
    _status.error = setpoint - measure;
    _status.i_term += _status.error * _ki * dT;
    _status.i_term = fboundf( -_iLimit, _iLimit, _status.i_term);

    float diff;

    diff = _status.error - _lastError;
    _lastError = _status.error;

    _status.d_term = 0;

    if (_kd > 0.0f && dT > 0.0f) {
        _status.d_term = diff * _kd / dT;
    }

    _status.i_min = fminf(_status.i_min, _status.i_term);
    _status.i_max = fmaxf(_status.i_max, _status.i_term);
    _status.p_term = (_status.error * _kp);

    _status.output =  _status.p_term + _status.i_term + _status.d_term;
    _status.output = fboundf(_min, _max, _status.output);
    float output = _status.output;
    release();
    return output;
}
} // namespace unav::controls