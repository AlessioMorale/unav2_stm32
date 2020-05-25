#ifndef _ROS_unav2_msgs_Diagnostic_h
#define _ROS_unav2_msgs_Diagnostic_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "unav2_msgs/PerfCounter.h"

namespace unav2_msgs
{

  class Diagnostic : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      uint32_t counters_length;
      typedef unav2_msgs::PerfCounter _counters_type;
      _counters_type st_counters;
      _counters_type * counters;

    Diagnostic():
      stamp(),
      counters_length(0), counters(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      *(outbuffer + offset + 0) = (this->counters_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->counters_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->counters_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->counters_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->counters_length);
      for( uint32_t i = 0; i < counters_length; i++){
      offset += this->counters[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      uint32_t counters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      counters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      counters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      counters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->counters_length);
      if(counters_lengthT > counters_length)
        this->counters = (unav2_msgs::PerfCounter*)realloc(this->counters, counters_lengthT * sizeof(unav2_msgs::PerfCounter));
      counters_length = counters_lengthT;
      for( uint32_t i = 0; i < counters_length; i++){
      offset += this->st_counters.deserialize(inbuffer + offset);
        memcpy( &(this->counters[i]), &(this->st_counters), sizeof(unav2_msgs::PerfCounter));
      }
     return offset;
    }

    const char * getType(){ return "unav2_msgs/Diagnostic"; };
    const char * getMD5(){ return "401934a5ed4f07c3116ba11ef75ab674"; };

  };

}
#endif
