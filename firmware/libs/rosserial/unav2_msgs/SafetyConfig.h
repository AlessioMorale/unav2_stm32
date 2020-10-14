#ifndef _ROS_unav2_msgs_SafetyConfig_h
#define _ROS_unav2_msgs_SafetyConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class SafetyConfig : public ros::Msg
  {
    public:
      typedef float _temp_warning_type;
      _temp_warning_type temp_warning;
      typedef float _temp_limit_type;
      _temp_limit_type temp_limit;
      typedef int16_t _temp_timeout_type;
      _temp_timeout_type temp_timeout;
      typedef int16_t _temp_autorestore_type;
      _temp_autorestore_type temp_autorestore;
      typedef float _current_warning_type;
      _current_warning_type current_warning;
      typedef float _current_limit_type;
      _current_limit_type current_limit;
      typedef int16_t _current_timeout_type;
      _current_timeout_type current_timeout;
      typedef int16_t _current_autorestore_type;
      _current_autorestore_type current_autorestore;
      typedef float _position_limit_type;
      _position_limit_type position_limit;
      typedef float _velocity_limit_type;
      _velocity_limit_type velocity_limit;
      typedef float _max_acceleration_type;
      _max_acceleration_type max_acceleration;
      typedef float _max_deceleration_type;
      _max_deceleration_type max_deceleration;
      typedef int8_t _pwm_limit_type;
      _pwm_limit_type pwm_limit;
      typedef int8_t _error_limit_type;
      _error_limit_type error_limit;
      typedef int16_t _slope_time_type;
      _slope_time_type slope_time;
      typedef int16_t _bridge_off_type;
      _bridge_off_type bridge_off;
      typedef int16_t _timeout_type;
      _timeout_type timeout;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;

    SafetyConfig():
      temp_warning(0),
      temp_limit(0),
      temp_timeout(0),
      temp_autorestore(0),
      current_warning(0),
      current_limit(0),
      current_timeout(0),
      current_autorestore(0),
      position_limit(0),
      velocity_limit(0),
      max_acceleration(0),
      max_deceleration(0),
      pwm_limit(0),
      error_limit(0),
      slope_time(0),
      bridge_off(0),
      timeout(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_temp_warning;
      u_temp_warning.real = this->temp_warning;
      *(outbuffer + offset + 0) = (u_temp_warning.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_warning.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_warning.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_warning.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_warning);
      union {
        float real;
        uint32_t base;
      } u_temp_limit;
      u_temp_limit.real = this->temp_limit;
      *(outbuffer + offset + 0) = (u_temp_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_limit);
      union {
        int16_t real;
        uint16_t base;
      } u_temp_timeout;
      u_temp_timeout.real = this->temp_timeout;
      *(outbuffer + offset + 0) = (u_temp_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_timeout.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->temp_timeout);
      union {
        int16_t real;
        uint16_t base;
      } u_temp_autorestore;
      u_temp_autorestore.real = this->temp_autorestore;
      *(outbuffer + offset + 0) = (u_temp_autorestore.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_autorestore.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->temp_autorestore);
      union {
        float real;
        uint32_t base;
      } u_current_warning;
      u_current_warning.real = this->current_warning;
      *(outbuffer + offset + 0) = (u_current_warning.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_warning.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_warning.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_warning.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_warning);
      union {
        float real;
        uint32_t base;
      } u_current_limit;
      u_current_limit.real = this->current_limit;
      *(outbuffer + offset + 0) = (u_current_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_limit);
      union {
        int16_t real;
        uint16_t base;
      } u_current_timeout;
      u_current_timeout.real = this->current_timeout;
      *(outbuffer + offset + 0) = (u_current_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_timeout.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_timeout);
      union {
        int16_t real;
        uint16_t base;
      } u_current_autorestore;
      u_current_autorestore.real = this->current_autorestore;
      *(outbuffer + offset + 0) = (u_current_autorestore.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_autorestore.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_autorestore);
      union {
        float real;
        uint32_t base;
      } u_position_limit;
      u_position_limit.real = this->position_limit;
      *(outbuffer + offset + 0) = (u_position_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_limit);
      union {
        float real;
        uint32_t base;
      } u_velocity_limit;
      u_velocity_limit.real = this->velocity_limit;
      *(outbuffer + offset + 0) = (u_velocity_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_limit);
      union {
        float real;
        uint32_t base;
      } u_max_acceleration;
      u_max_acceleration.real = this->max_acceleration;
      *(outbuffer + offset + 0) = (u_max_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_acceleration);
      union {
        float real;
        uint32_t base;
      } u_max_deceleration;
      u_max_deceleration.real = this->max_deceleration;
      *(outbuffer + offset + 0) = (u_max_deceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_deceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_deceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_deceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_deceleration);
      union {
        int8_t real;
        uint8_t base;
      } u_pwm_limit;
      u_pwm_limit.real = this->pwm_limit;
      *(outbuffer + offset + 0) = (u_pwm_limit.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pwm_limit);
      union {
        int8_t real;
        uint8_t base;
      } u_error_limit;
      u_error_limit.real = this->error_limit;
      *(outbuffer + offset + 0) = (u_error_limit.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->error_limit);
      union {
        int16_t real;
        uint16_t base;
      } u_slope_time;
      u_slope_time.real = this->slope_time;
      *(outbuffer + offset + 0) = (u_slope_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_slope_time.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->slope_time);
      union {
        int16_t real;
        uint16_t base;
      } u_bridge_off;
      u_bridge_off.real = this->bridge_off;
      *(outbuffer + offset + 0) = (u_bridge_off.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bridge_off.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->bridge_off);
      union {
        int16_t real;
        uint16_t base;
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timeout.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->timeout);
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
      } u_temp_warning;
      u_temp_warning.base = 0;
      u_temp_warning.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_warning.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_warning.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_warning.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_warning = u_temp_warning.real;
      offset += sizeof(this->temp_warning);
      union {
        float real;
        uint32_t base;
      } u_temp_limit;
      u_temp_limit.base = 0;
      u_temp_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_limit = u_temp_limit.real;
      offset += sizeof(this->temp_limit);
      union {
        int16_t real;
        uint16_t base;
      } u_temp_timeout;
      u_temp_timeout.base = 0;
      u_temp_timeout.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_timeout.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->temp_timeout = u_temp_timeout.real;
      offset += sizeof(this->temp_timeout);
      union {
        int16_t real;
        uint16_t base;
      } u_temp_autorestore;
      u_temp_autorestore.base = 0;
      u_temp_autorestore.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_autorestore.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->temp_autorestore = u_temp_autorestore.real;
      offset += sizeof(this->temp_autorestore);
      union {
        float real;
        uint32_t base;
      } u_current_warning;
      u_current_warning.base = 0;
      u_current_warning.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_warning.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_warning.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_warning.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_warning = u_current_warning.real;
      offset += sizeof(this->current_warning);
      union {
        float real;
        uint32_t base;
      } u_current_limit;
      u_current_limit.base = 0;
      u_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_limit = u_current_limit.real;
      offset += sizeof(this->current_limit);
      union {
        int16_t real;
        uint16_t base;
      } u_current_timeout;
      u_current_timeout.base = 0;
      u_current_timeout.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_timeout.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_timeout = u_current_timeout.real;
      offset += sizeof(this->current_timeout);
      union {
        int16_t real;
        uint16_t base;
      } u_current_autorestore;
      u_current_autorestore.base = 0;
      u_current_autorestore.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_autorestore.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_autorestore = u_current_autorestore.real;
      offset += sizeof(this->current_autorestore);
      union {
        float real;
        uint32_t base;
      } u_position_limit;
      u_position_limit.base = 0;
      u_position_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_limit = u_position_limit.real;
      offset += sizeof(this->position_limit);
      union {
        float real;
        uint32_t base;
      } u_velocity_limit;
      u_velocity_limit.base = 0;
      u_velocity_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_limit = u_velocity_limit.real;
      offset += sizeof(this->velocity_limit);
      union {
        float real;
        uint32_t base;
      } u_max_acceleration;
      u_max_acceleration.base = 0;
      u_max_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_acceleration = u_max_acceleration.real;
      offset += sizeof(this->max_acceleration);
      union {
        float real;
        uint32_t base;
      } u_max_deceleration;
      u_max_deceleration.base = 0;
      u_max_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_deceleration = u_max_deceleration.real;
      offset += sizeof(this->max_deceleration);
      union {
        int8_t real;
        uint8_t base;
      } u_pwm_limit;
      u_pwm_limit.base = 0;
      u_pwm_limit.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pwm_limit = u_pwm_limit.real;
      offset += sizeof(this->pwm_limit);
      union {
        int8_t real;
        uint8_t base;
      } u_error_limit;
      u_error_limit.base = 0;
      u_error_limit.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->error_limit = u_error_limit.real;
      offset += sizeof(this->error_limit);
      union {
        int16_t real;
        uint16_t base;
      } u_slope_time;
      u_slope_time.base = 0;
      u_slope_time.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_slope_time.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->slope_time = u_slope_time.real;
      offset += sizeof(this->slope_time);
      union {
        int16_t real;
        uint16_t base;
      } u_bridge_off;
      u_bridge_off.base = 0;
      u_bridge_off.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bridge_off.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->bridge_off = u_bridge_off.real;
      offset += sizeof(this->bridge_off);
      union {
        int16_t real;
        uint16_t base;
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timeout.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/SafetyConfig"; };
    const char * getMD5(){ return "d90ade4da44f6914a1c08c1d3149dd3f"; };

  };

}
#endif
