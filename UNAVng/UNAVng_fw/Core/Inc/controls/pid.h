/*
** unav2
** File description:
** pid
*/

#pragma once
#include <FreeRTOS.h>
#include <semphr.h>
namespace unav::controls {
  typedef struct _pid_status {
    float timestep;
    float error;
    float p_term;
    float i_term;
    float d_term;
    float i_max;
    float i_min;
    float output;
  } pid_status_t;

class PID {
public:
  PID();
  void setGains(float kp, float ki, float kd, float ilimit);
  float apply(float setpoint, float measure, float dT);
  void zero();
  void setRange(float min, float max);
  void getStatus(pid_status_t *status);
private:
  inline void lock(){ xSemaphoreTake(_mutex, portMAX_DELAY);}
  inline void release(){ xSemaphoreGive(_mutex);}
  float _kp;
  float _ki;
  float _kd;
  float _iLimit;
  float _iAccumulator;
  float _lastError;
  float _min;
  float _max;
  StaticSemaphore_t _semBuffer;
  SemaphoreHandle_t _mutex; 
  pid_status_t _status;
};
} // namespace unav::controls
