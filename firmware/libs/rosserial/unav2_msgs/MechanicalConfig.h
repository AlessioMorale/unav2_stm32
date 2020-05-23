#ifndef _ROS_unav2_msgs_MechanicalConfig_h
#define _ROS_unav2_msgs_MechanicalConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class MechanicalConfig : public ros::Msg
  {
    public:
      typedef float _ratio_type;
      _ratio_type ratio;
      typedef bool _rotation0_type;
      _rotation0_type rotation0;
      typedef bool _rotation1_type;
      _rotation1_type rotation1;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;

    MechanicalConfig():
      ratio(0),
      rotation0(0),
      rotation1(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ratio;
      u_ratio.real = this->ratio;
      *(outbuffer + offset + 0) = (u_ratio.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ratio.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ratio.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ratio.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ratio);
      union {
        bool real;
        uint8_t base;
      } u_rotation0;
      u_rotation0.real = this->rotation0;
      *(outbuffer + offset + 0) = (u_rotation0.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rotation0);
      union {
        bool real;
        uint8_t base;
      } u_rotation1;
      u_rotation1.real = this->rotation1;
      *(outbuffer + offset + 0) = (u_rotation1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rotation1);
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
      } u_ratio;
      u_ratio.base = 0;
      u_ratio.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ratio.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ratio.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ratio.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ratio = u_ratio.real;
      offset += sizeof(this->ratio);
      union {
        bool real;
        uint8_t base;
      } u_rotation0;
      u_rotation0.base = 0;
      u_rotation0.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rotation0 = u_rotation0.real;
      offset += sizeof(this->rotation0);
      union {
        bool real;
        uint8_t base;
      } u_rotation1;
      u_rotation1.base = 0;
      u_rotation1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rotation1 = u_rotation1.real;
      offset += sizeof(this->rotation1);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/MechanicalConfig"; };
    const char * getMD5(){ return "79af9aebe53ea8303292896b6acd440a"; };

  };

}
#endif
