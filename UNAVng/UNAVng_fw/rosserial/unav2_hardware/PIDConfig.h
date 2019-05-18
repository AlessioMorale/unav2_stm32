#ifndef _ROS_unav2_hardware_PIDConfig_h
#define _ROS_unav2_hardware_PIDConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_hardware
{

  class PIDConfig : public ros::Msg
  {
    public:
      typedef float _vel_kp_type;
      _vel_kp_type vel_kp;
      typedef float _vel_ki_type;
      _vel_ki_type vel_ki;
      typedef float _vel_kd_type;
      _vel_kd_type vel_kd;
      typedef float _vel_kaw_type;
      _vel_kaw_type vel_kaw;
      typedef float _cur_kp_type;
      _cur_kp_type cur_kp;
      typedef float _cur_ki_type;
      _cur_ki_type cur_ki;
      typedef float _cur_kd_type;
      _cur_kd_type cur_kd;
      typedef float _cur_kaw_type;
      _cur_kaw_type cur_kaw;
      typedef uint16_t _vel_frequency_type;
      _vel_frequency_type vel_frequency;
      typedef uint16_t _cur_frequency_type;
      _cur_frequency_type cur_frequency;
      typedef bool _cur_enable_type;
      _cur_enable_type cur_enable;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;

    PIDConfig():
      vel_kp(0),
      vel_ki(0),
      vel_kd(0),
      vel_kaw(0),
      cur_kp(0),
      cur_ki(0),
      cur_kd(0),
      cur_kaw(0),
      vel_frequency(0),
      cur_frequency(0),
      cur_enable(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_vel_kp;
      u_vel_kp.real = this->vel_kp;
      *(outbuffer + offset + 0) = (u_vel_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_kp);
      union {
        float real;
        uint32_t base;
      } u_vel_ki;
      u_vel_ki.real = this->vel_ki;
      *(outbuffer + offset + 0) = (u_vel_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_ki);
      union {
        float real;
        uint32_t base;
      } u_vel_kd;
      u_vel_kd.real = this->vel_kd;
      *(outbuffer + offset + 0) = (u_vel_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_kd);
      union {
        float real;
        uint32_t base;
      } u_vel_kaw;
      u_vel_kaw.real = this->vel_kaw;
      *(outbuffer + offset + 0) = (u_vel_kaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_kaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_kaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_kaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_kaw);
      union {
        float real;
        uint32_t base;
      } u_cur_kp;
      u_cur_kp.real = this->cur_kp;
      *(outbuffer + offset + 0) = (u_cur_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cur_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cur_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cur_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cur_kp);
      union {
        float real;
        uint32_t base;
      } u_cur_ki;
      u_cur_ki.real = this->cur_ki;
      *(outbuffer + offset + 0) = (u_cur_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cur_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cur_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cur_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cur_ki);
      union {
        float real;
        uint32_t base;
      } u_cur_kd;
      u_cur_kd.real = this->cur_kd;
      *(outbuffer + offset + 0) = (u_cur_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cur_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cur_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cur_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cur_kd);
      union {
        float real;
        uint32_t base;
      } u_cur_kaw;
      u_cur_kaw.real = this->cur_kaw;
      *(outbuffer + offset + 0) = (u_cur_kaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cur_kaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cur_kaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cur_kaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cur_kaw);
      *(outbuffer + offset + 0) = (this->vel_frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vel_frequency >> (8 * 1)) & 0xFF;
      offset += sizeof(this->vel_frequency);
      *(outbuffer + offset + 0) = (this->cur_frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cur_frequency >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cur_frequency);
      union {
        bool real;
        uint8_t base;
      } u_cur_enable;
      u_cur_enable.real = this->cur_enable;
      *(outbuffer + offset + 0) = (u_cur_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cur_enable);
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
      } u_vel_kp;
      u_vel_kp.base = 0;
      u_vel_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_kp = u_vel_kp.real;
      offset += sizeof(this->vel_kp);
      union {
        float real;
        uint32_t base;
      } u_vel_ki;
      u_vel_ki.base = 0;
      u_vel_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_ki = u_vel_ki.real;
      offset += sizeof(this->vel_ki);
      union {
        float real;
        uint32_t base;
      } u_vel_kd;
      u_vel_kd.base = 0;
      u_vel_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_kd = u_vel_kd.real;
      offset += sizeof(this->vel_kd);
      union {
        float real;
        uint32_t base;
      } u_vel_kaw;
      u_vel_kaw.base = 0;
      u_vel_kaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_kaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_kaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_kaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_kaw = u_vel_kaw.real;
      offset += sizeof(this->vel_kaw);
      union {
        float real;
        uint32_t base;
      } u_cur_kp;
      u_cur_kp.base = 0;
      u_cur_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cur_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cur_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cur_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cur_kp = u_cur_kp.real;
      offset += sizeof(this->cur_kp);
      union {
        float real;
        uint32_t base;
      } u_cur_ki;
      u_cur_ki.base = 0;
      u_cur_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cur_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cur_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cur_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cur_ki = u_cur_ki.real;
      offset += sizeof(this->cur_ki);
      union {
        float real;
        uint32_t base;
      } u_cur_kd;
      u_cur_kd.base = 0;
      u_cur_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cur_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cur_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cur_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cur_kd = u_cur_kd.real;
      offset += sizeof(this->cur_kd);
      union {
        float real;
        uint32_t base;
      } u_cur_kaw;
      u_cur_kaw.base = 0;
      u_cur_kaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cur_kaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cur_kaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cur_kaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cur_kaw = u_cur_kaw.real;
      offset += sizeof(this->cur_kaw);
      this->vel_frequency =  ((uint16_t) (*(inbuffer + offset)));
      this->vel_frequency |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->vel_frequency);
      this->cur_frequency =  ((uint16_t) (*(inbuffer + offset)));
      this->cur_frequency |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->cur_frequency);
      union {
        bool real;
        uint8_t base;
      } u_cur_enable;
      u_cur_enable.base = 0;
      u_cur_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cur_enable = u_cur_enable.real;
      offset += sizeof(this->cur_enable);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_hardware/PIDConfig"; };
    const char * getMD5(){ return "5dbac42a27ccdf4718ac83386a098f08"; };

  };

}
#endif
