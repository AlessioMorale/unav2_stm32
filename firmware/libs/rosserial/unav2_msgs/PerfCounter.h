#ifndef _ROS_unav2_msgs_PerfCounter_h
#define _ROS_unav2_msgs_PerfCounter_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class PerfCounter : public ros::Msg
  {
    public:
      typedef const char* _key_type;
      _key_type key;
      typedef int32_t _value_type;
      _value_type value;
      typedef int32_t _max_type;
      _max_type max;
      typedef int32_t _min_type;
      _min_type min;
      typedef uint32_t _lastUpdateTS_type;
      _lastUpdateTS_type lastUpdateTS;

    PerfCounter():
      key(""),
      value(0),
      max(0),
      min(0),
      lastUpdateTS(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_key = strlen(this->key);
      varToArr(outbuffer + offset, length_key);
      offset += 4;
      memcpy(outbuffer + offset, this->key, length_key);
      offset += length_key;
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      union {
        int32_t real;
        uint32_t base;
      } u_max;
      u_max.real = this->max;
      *(outbuffer + offset + 0) = (u_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max);
      union {
        int32_t real;
        uint32_t base;
      } u_min;
      u_min.real = this->min;
      *(outbuffer + offset + 0) = (u_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min);
      *(outbuffer + offset + 0) = (this->lastUpdateTS >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lastUpdateTS >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lastUpdateTS >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lastUpdateTS >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lastUpdateTS);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_key;
      arrToVar(length_key, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_key; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_key-1]=0;
      this->key = (char *)(inbuffer + offset-1);
      offset += length_key;
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value = u_value.real;
      offset += sizeof(this->value);
      union {
        int32_t real;
        uint32_t base;
      } u_max;
      u_max.base = 0;
      u_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max = u_max.real;
      offset += sizeof(this->max);
      union {
        int32_t real;
        uint32_t base;
      } u_min;
      u_min.base = 0;
      u_min.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min = u_min.real;
      offset += sizeof(this->min);
      this->lastUpdateTS =  ((uint32_t) (*(inbuffer + offset)));
      this->lastUpdateTS |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->lastUpdateTS |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->lastUpdateTS |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->lastUpdateTS);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/PerfCounter"; };
    const char * getMD5(){ return "8e1d11c309894b796a88d24c3501e374"; };

  };

}
#endif
