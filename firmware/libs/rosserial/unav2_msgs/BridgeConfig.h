#ifndef _ROS_unav2_msgs_BridgeConfig_h
#define _ROS_unav2_msgs_BridgeConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class BridgeConfig : public ros::Msg
  {
    public:
      typedef uint16_t _pwm_dead_time_type;
      _pwm_dead_time_type pwm_dead_time;
      typedef uint16_t _pwm_frequency_type;
      _pwm_frequency_type pwm_frequency;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;

    BridgeConfig():
      pwm_dead_time(0),
      pwm_frequency(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pwm_dead_time >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pwm_dead_time >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pwm_dead_time);
      *(outbuffer + offset + 0) = (this->pwm_frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pwm_frequency >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pwm_frequency);
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
      this->pwm_dead_time =  ((uint16_t) (*(inbuffer + offset)));
      this->pwm_dead_time |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pwm_dead_time);
      this->pwm_frequency =  ((uint16_t) (*(inbuffer + offset)));
      this->pwm_frequency |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pwm_frequency);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/BridgeConfig"; };
    const char * getMD5(){ return "461351cb47c5a04d582152dd9aaecf0e"; };

  };

}
#endif
