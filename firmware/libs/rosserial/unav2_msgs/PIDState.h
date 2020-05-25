#ifndef _ROS_unav2_msgs_PIDState_h
#define _ROS_unav2_msgs_PIDState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace unav2_msgs
{

  class PIDState : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      uint32_t timestep_length;
      typedef float _timestep_type;
      _timestep_type st_timestep;
      _timestep_type * timestep;
      uint32_t error_length;
      typedef float _error_type;
      _error_type st_error;
      _error_type * error;
      uint32_t p_term_length;
      typedef float _p_term_type;
      _p_term_type st_p_term;
      _p_term_type * p_term;
      uint32_t i_term_length;
      typedef float _i_term_type;
      _i_term_type st_i_term;
      _i_term_type * i_term;
      uint32_t d_term_length;
      typedef float _d_term_type;
      _d_term_type st_d_term;
      _d_term_type * d_term;
      uint32_t i_max_length;
      typedef float _i_max_type;
      _i_max_type st_i_max;
      _i_max_type * i_max;
      uint32_t i_min_length;
      typedef float _i_min_type;
      _i_min_type st_i_min;
      _i_min_type * i_min;
      uint32_t output_length;
      typedef float _output_type;
      _output_type st_output;
      _output_type * output;

    PIDState():
      stamp(),
      timestep_length(0), timestep(NULL),
      error_length(0), error(NULL),
      p_term_length(0), p_term(NULL),
      i_term_length(0), i_term(NULL),
      d_term_length(0), d_term(NULL),
      i_max_length(0), i_max(NULL),
      i_min_length(0), i_min(NULL),
      output_length(0), output(NULL)
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
      *(outbuffer + offset + 0) = (this->timestep_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestep_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestep_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestep_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestep_length);
      for( uint32_t i = 0; i < timestep_length; i++){
      union {
        float real;
        uint32_t base;
      } u_timestepi;
      u_timestepi.real = this->timestep[i];
      *(outbuffer + offset + 0) = (u_timestepi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timestepi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timestepi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timestepi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestep[i]);
      }
      *(outbuffer + offset + 0) = (this->error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error_length);
      for( uint32_t i = 0; i < error_length; i++){
      union {
        float real;
        uint32_t base;
      } u_errori;
      u_errori.real = this->error[i];
      *(outbuffer + offset + 0) = (u_errori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_errori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_errori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_errori.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error[i]);
      }
      *(outbuffer + offset + 0) = (this->p_term_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->p_term_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->p_term_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->p_term_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_term_length);
      for( uint32_t i = 0; i < p_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_p_termi;
      u_p_termi.real = this->p_term[i];
      *(outbuffer + offset + 0) = (u_p_termi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p_termi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p_termi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p_termi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_term[i]);
      }
      *(outbuffer + offset + 0) = (this->i_term_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_term_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i_term_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i_term_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_term_length);
      for( uint32_t i = 0; i < i_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_i_termi;
      u_i_termi.real = this->i_term[i];
      *(outbuffer + offset + 0) = (u_i_termi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_termi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_termi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_termi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_term[i]);
      }
      *(outbuffer + offset + 0) = (this->d_term_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->d_term_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->d_term_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->d_term_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d_term_length);
      for( uint32_t i = 0; i < d_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_d_termi;
      u_d_termi.real = this->d_term[i];
      *(outbuffer + offset + 0) = (u_d_termi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d_termi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d_termi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d_termi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d_term[i]);
      }
      *(outbuffer + offset + 0) = (this->i_max_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_max_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i_max_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i_max_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_max_length);
      for( uint32_t i = 0; i < i_max_length; i++){
      union {
        float real;
        uint32_t base;
      } u_i_maxi;
      u_i_maxi.real = this->i_max[i];
      *(outbuffer + offset + 0) = (u_i_maxi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_maxi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_maxi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_maxi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_max[i]);
      }
      *(outbuffer + offset + 0) = (this->i_min_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_min_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i_min_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i_min_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_min_length);
      for( uint32_t i = 0; i < i_min_length; i++){
      union {
        float real;
        uint32_t base;
      } u_i_mini;
      u_i_mini.real = this->i_min[i];
      *(outbuffer + offset + 0) = (u_i_mini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_mini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_mini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_mini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_min[i]);
      }
      *(outbuffer + offset + 0) = (this->output_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->output_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->output_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->output_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_length);
      for( uint32_t i = 0; i < output_length; i++){
      union {
        float real;
        uint32_t base;
      } u_outputi;
      u_outputi.real = this->output[i];
      *(outbuffer + offset + 0) = (u_outputi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_outputi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_outputi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_outputi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output[i]);
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
      uint32_t timestep_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      timestep_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      timestep_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      timestep_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->timestep_length);
      if(timestep_lengthT > timestep_length)
        this->timestep = (float*)realloc(this->timestep, timestep_lengthT * sizeof(float));
      timestep_length = timestep_lengthT;
      for( uint32_t i = 0; i < timestep_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_timestep;
      u_st_timestep.base = 0;
      u_st_timestep.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_timestep.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_timestep.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_timestep.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_timestep = u_st_timestep.real;
      offset += sizeof(this->st_timestep);
        memcpy( &(this->timestep[i]), &(this->st_timestep), sizeof(float));
      }
      uint32_t error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->error_length);
      if(error_lengthT > error_length)
        this->error = (float*)realloc(this->error, error_lengthT * sizeof(float));
      error_length = error_lengthT;
      for( uint32_t i = 0; i < error_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_error;
      u_st_error.base = 0;
      u_st_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_error = u_st_error.real;
      offset += sizeof(this->st_error);
        memcpy( &(this->error[i]), &(this->st_error), sizeof(float));
      }
      uint32_t p_term_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      p_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      p_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      p_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->p_term_length);
      if(p_term_lengthT > p_term_length)
        this->p_term = (float*)realloc(this->p_term, p_term_lengthT * sizeof(float));
      p_term_length = p_term_lengthT;
      for( uint32_t i = 0; i < p_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_p_term;
      u_st_p_term.base = 0;
      u_st_p_term.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_p_term.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_p_term.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_p_term.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_p_term = u_st_p_term.real;
      offset += sizeof(this->st_p_term);
        memcpy( &(this->p_term[i]), &(this->st_p_term), sizeof(float));
      }
      uint32_t i_term_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      i_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      i_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      i_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->i_term_length);
      if(i_term_lengthT > i_term_length)
        this->i_term = (float*)realloc(this->i_term, i_term_lengthT * sizeof(float));
      i_term_length = i_term_lengthT;
      for( uint32_t i = 0; i < i_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_i_term;
      u_st_i_term.base = 0;
      u_st_i_term.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_i_term.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_i_term.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_i_term.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_i_term = u_st_i_term.real;
      offset += sizeof(this->st_i_term);
        memcpy( &(this->i_term[i]), &(this->st_i_term), sizeof(float));
      }
      uint32_t d_term_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      d_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      d_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      d_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->d_term_length);
      if(d_term_lengthT > d_term_length)
        this->d_term = (float*)realloc(this->d_term, d_term_lengthT * sizeof(float));
      d_term_length = d_term_lengthT;
      for( uint32_t i = 0; i < d_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_d_term;
      u_st_d_term.base = 0;
      u_st_d_term.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_d_term.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_d_term.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_d_term.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_d_term = u_st_d_term.real;
      offset += sizeof(this->st_d_term);
        memcpy( &(this->d_term[i]), &(this->st_d_term), sizeof(float));
      }
      uint32_t i_max_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      i_max_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      i_max_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      i_max_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->i_max_length);
      if(i_max_lengthT > i_max_length)
        this->i_max = (float*)realloc(this->i_max, i_max_lengthT * sizeof(float));
      i_max_length = i_max_lengthT;
      for( uint32_t i = 0; i < i_max_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_i_max;
      u_st_i_max.base = 0;
      u_st_i_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_i_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_i_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_i_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_i_max = u_st_i_max.real;
      offset += sizeof(this->st_i_max);
        memcpy( &(this->i_max[i]), &(this->st_i_max), sizeof(float));
      }
      uint32_t i_min_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      i_min_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      i_min_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      i_min_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->i_min_length);
      if(i_min_lengthT > i_min_length)
        this->i_min = (float*)realloc(this->i_min, i_min_lengthT * sizeof(float));
      i_min_length = i_min_lengthT;
      for( uint32_t i = 0; i < i_min_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_i_min;
      u_st_i_min.base = 0;
      u_st_i_min.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_i_min.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_i_min.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_i_min.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_i_min = u_st_i_min.real;
      offset += sizeof(this->st_i_min);
        memcpy( &(this->i_min[i]), &(this->st_i_min), sizeof(float));
      }
      uint32_t output_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      output_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      output_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      output_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->output_length);
      if(output_lengthT > output_length)
        this->output = (float*)realloc(this->output, output_lengthT * sizeof(float));
      output_length = output_lengthT;
      for( uint32_t i = 0; i < output_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_output;
      u_st_output.base = 0;
      u_st_output.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_output.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_output.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_output.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_output = u_st_output.real;
      offset += sizeof(this->st_output);
        memcpy( &(this->output[i]), &(this->st_output), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "unav2_msgs/PIDState"; };
    const char * getMD5(){ return "9f8773bbe78f298721ce0f900de179ff"; };

  };

}
#endif
