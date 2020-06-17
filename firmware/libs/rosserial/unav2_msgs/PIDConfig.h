#ifndef _ROS_unav2_msgs_PIDConfig_h
#define _ROS_unav2_msgs_PIDConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class PIDConfig : public ros::Msg
  {
    public:
      typedef float _velocity_kp_type;
      _velocity_kp_type velocity_kp;
      typedef float _velocity_ki_type;
      _velocity_ki_type velocity_ki;
      typedef float _velocity_kd_type;
      _velocity_kd_type velocity_kd;
      typedef float _velocity_kaw_type;
      _velocity_kaw_type velocity_kaw;
      typedef float _current_kp_type;
      _current_kp_type current_kp;
      typedef float _current_ki_type;
      _current_ki_type current_ki;
      typedef float _current_kd_type;
      _current_kd_type current_kd;
      typedef float _current_kaw_type;
      _current_kaw_type current_kaw;
      typedef uint16_t _velocity_frequency_type;
      _velocity_frequency_type velocity_frequency;
      typedef uint16_t _current_frequency_type;
      _current_frequency_type current_frequency;
      typedef bool _current_enable_type;
      _current_enable_type current_enable;
      typedef bool _pid_debug_type;
      _pid_debug_type pid_debug;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;

    PIDConfig():
      velocity_kp(0),
      velocity_ki(0),
      velocity_kd(0),
      velocity_kaw(0),
      current_kp(0),
      current_ki(0),
      current_kd(0),
      current_kaw(0),
      velocity_frequency(0),
      current_frequency(0),
      current_enable(0),
      pid_debug(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_velocity_kp;
      u_velocity_kp.real = this->velocity_kp;
      *(outbuffer + offset + 0) = (u_velocity_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_kp);
      union {
        float real;
        uint32_t base;
      } u_velocity_ki;
      u_velocity_ki.real = this->velocity_ki;
      *(outbuffer + offset + 0) = (u_velocity_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_ki);
      union {
        float real;
        uint32_t base;
      } u_velocity_kd;
      u_velocity_kd.real = this->velocity_kd;
      *(outbuffer + offset + 0) = (u_velocity_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_kd);
      union {
        float real;
        uint32_t base;
      } u_velocity_kaw;
      u_velocity_kaw.real = this->velocity_kaw;
      *(outbuffer + offset + 0) = (u_velocity_kaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_kaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_kaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_kaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_kaw);
      union {
        float real;
        uint32_t base;
      } u_current_kp;
      u_current_kp.real = this->current_kp;
      *(outbuffer + offset + 0) = (u_current_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_kp);
      union {
        float real;
        uint32_t base;
      } u_current_ki;
      u_current_ki.real = this->current_ki;
      *(outbuffer + offset + 0) = (u_current_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_ki);
      union {
        float real;
        uint32_t base;
      } u_current_kd;
      u_current_kd.real = this->current_kd;
      *(outbuffer + offset + 0) = (u_current_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_kd);
      union {
        float real;
        uint32_t base;
      } u_current_kaw;
      u_current_kaw.real = this->current_kaw;
      *(outbuffer + offset + 0) = (u_current_kaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_kaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_kaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_kaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_kaw);
      *(outbuffer + offset + 0) = (this->velocity_frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_frequency >> (8 * 1)) & 0xFF;
      offset += sizeof(this->velocity_frequency);
      *(outbuffer + offset + 0) = (this->current_frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_frequency >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_frequency);
      union {
        bool real;
        uint8_t base;
      } u_current_enable;
      u_current_enable.real = this->current_enable;
      *(outbuffer + offset + 0) = (u_current_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current_enable);
      union {
        bool real;
        uint8_t base;
      } u_pid_debug;
      u_pid_debug.real = this->pid_debug;
      *(outbuffer + offset + 0) = (u_pid_debug.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pid_debug);
      *(outbuffer + offset + 0) = (this->transactionId >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transactionId >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transactionId >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transactionId >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transactionId);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_velocity_kp;
      u_velocity_kp.base = 0;
      u_velocity_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_kp = u_velocity_kp.real;
      offset += sizeof(this->velocity_kp);
      union {
        float real;
        uint32_t base;
      } u_velocity_ki;
      u_velocity_ki.base = 0;
      u_velocity_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_ki = u_velocity_ki.real;
      offset += sizeof(this->velocity_ki);
      union {
        float real;
        uint32_t base;
      } u_velocity_kd;
      u_velocity_kd.base = 0;
      u_velocity_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_kd = u_velocity_kd.real;
      offset += sizeof(this->velocity_kd);
      union {
        float real;
        uint32_t base;
      } u_velocity_kaw;
      u_velocity_kaw.base = 0;
      u_velocity_kaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_kaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_kaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_kaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_kaw = u_velocity_kaw.real;
      offset += sizeof(this->velocity_kaw);
      union {
        float real;
        uint32_t base;
      } u_current_kp;
      u_current_kp.base = 0;
      u_current_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_kp = u_current_kp.real;
      offset += sizeof(this->current_kp);
      union {
        float real;
        uint32_t base;
      } u_current_ki;
      u_current_ki.base = 0;
      u_current_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_ki = u_current_ki.real;
      offset += sizeof(this->current_ki);
      union {
        float real;
        uint32_t base;
      } u_current_kd;
      u_current_kd.base = 0;
      u_current_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_kd = u_current_kd.real;
      offset += sizeof(this->current_kd);
      union {
        float real;
        uint32_t base;
      } u_current_kaw;
      u_current_kaw.base = 0;
      u_current_kaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_kaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_kaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_kaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_kaw = u_current_kaw.real;
      offset += sizeof(this->current_kaw);
      this->velocity_frequency =  ((uint16_t) (*(inbuffer + offset)));
      this->velocity_frequency |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->velocity_frequency);
      this->current_frequency =  ((uint16_t) (*(inbuffer + offset)));
      this->current_frequency |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->current_frequency);
      union {
        bool real;
        uint8_t base;
      } u_current_enable;
      u_current_enable.base = 0;
      u_current_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->current_enable = u_current_enable.real;
      offset += sizeof(this->current_enable);
      union {
        bool real;
        uint8_t base;
      } u_pid_debug;
      u_pid_debug.base = 0;
      u_pid_debug.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pid_debug = u_pid_debug.real;
      offset += sizeof(this->pid_debug);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/PIDConfig"; };
    const char * getMD5(){ return "1ac4b0f39b5f14b7754252250417f891"; };

  };

}
#endif
