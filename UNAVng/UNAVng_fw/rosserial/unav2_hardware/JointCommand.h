#ifndef _ROS_unav2_hardware_JointCommand_h
#define _ROS_unav2_hardware_JointCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace unav2_hardware
{

  class JointCommand : public ros::Msg
  {
    public:
      typedef uint16_t _seq_type;
      _seq_type seq;
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef int8_t _commandMode_type;
      _commandMode_type commandMode;
      typedef uint8_t _count_type;
      _count_type count;
      uint32_t command_length;
      typedef float _command_type;
      _command_type st_command;
      _command_type * command;
      enum { FAILSAFE =  -1 };
      enum { POSITION =  1 };
      enum { VELOCITY =  2 };
      enum { EFFORT =  3 };
      enum { INVALID =  4 };

    JointCommand():
      seq(0),
      stamp(),
      commandMode(0),
      count(0),
      command_length(0), command(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->seq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq >> (8 * 1)) & 0xFF;
      offset += sizeof(this->seq);
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
      union {
        int8_t real;
        uint8_t base;
      } u_commandMode;
      u_commandMode.real = this->commandMode;
      *(outbuffer + offset + 0) = (u_commandMode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->commandMode);
      *(outbuffer + offset + 0) = (this->count >> (8 * 0)) & 0xFF;
      offset += sizeof(this->count);
      *(outbuffer + offset + 0) = (this->command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command_length);
      for( uint32_t i = 0; i < command_length; i++){
      union {
        float real;
        uint32_t base;
      } u_commandi;
      u_commandi.real = this->command[i];
      *(outbuffer + offset + 0) = (u_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_commandi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->seq =  ((uint16_t) (*(inbuffer + offset)));
      this->seq |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->seq);
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
      union {
        int8_t real;
        uint8_t base;
      } u_commandMode;
      u_commandMode.base = 0;
      u_commandMode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->commandMode = u_commandMode.real;
      offset += sizeof(this->commandMode);
      this->count =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->count);
      uint32_t command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->command_length);
      if(command_lengthT > command_length)
        this->command = (float*)realloc(this->command, command_lengthT * sizeof(float));
      command_length = command_lengthT;
      for( uint32_t i = 0; i < command_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_command;
      u_st_command.base = 0;
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_command = u_st_command.real;
      offset += sizeof(this->st_command);
        memcpy( &(this->command[i]), &(this->st_command), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "unav2_hardware/JointCommand"; };
    const char * getMD5(){ return "b690170aad5c2f463c0d22d3d94a8ed4"; };

  };

}
#endif
