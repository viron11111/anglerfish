#ifndef _ROS_sub8_msgs_Alarm_h
#define _ROS_sub8_msgs_Alarm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sub8_msgs
{

  class Alarm : public ros::Msg
  {
    public:
      std_msgs::Header header;
      bool action_required;
      const char* problem_description;
      const char* parameters;
      uint8_t severity;
      const char* alarm_name;
      const char* node_name;
      bool clear;

    Alarm():
      header(),
      action_required(0),
      problem_description(""),
      parameters(""),
      severity(0),
      alarm_name(""),
      node_name(""),
      clear(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_action_required;
      u_action_required.real = this->action_required;
      *(outbuffer + offset + 0) = (u_action_required.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->action_required);
      uint32_t length_problem_description = strlen(this->problem_description);
      memcpy(outbuffer + offset, &length_problem_description, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->problem_description, length_problem_description);
      offset += length_problem_description;
      uint32_t length_parameters = strlen(this->parameters);
      memcpy(outbuffer + offset, &length_parameters, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->parameters, length_parameters);
      offset += length_parameters;
      *(outbuffer + offset + 0) = (this->severity >> (8 * 0)) & 0xFF;
      offset += sizeof(this->severity);
      uint32_t length_alarm_name = strlen(this->alarm_name);
      memcpy(outbuffer + offset, &length_alarm_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->alarm_name, length_alarm_name);
      offset += length_alarm_name;
      uint32_t length_node_name = strlen(this->node_name);
      memcpy(outbuffer + offset, &length_node_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->node_name, length_node_name);
      offset += length_node_name;
      union {
        bool real;
        uint8_t base;
      } u_clear;
      u_clear.real = this->clear;
      *(outbuffer + offset + 0) = (u_clear.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clear);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_action_required;
      u_action_required.base = 0;
      u_action_required.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->action_required = u_action_required.real;
      offset += sizeof(this->action_required);
      uint32_t length_problem_description;
      memcpy(&length_problem_description, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_problem_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_problem_description-1]=0;
      this->problem_description = (char *)(inbuffer + offset-1);
      offset += length_problem_description;
      uint32_t length_parameters;
      memcpy(&length_parameters, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parameters; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parameters-1]=0;
      this->parameters = (char *)(inbuffer + offset-1);
      offset += length_parameters;
      this->severity =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->severity);
      uint32_t length_alarm_name;
      memcpy(&length_alarm_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_alarm_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_alarm_name-1]=0;
      this->alarm_name = (char *)(inbuffer + offset-1);
      offset += length_alarm_name;
      uint32_t length_node_name;
      memcpy(&length_node_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node_name-1]=0;
      this->node_name = (char *)(inbuffer + offset-1);
      offset += length_node_name;
      union {
        bool real;
        uint8_t base;
      } u_clear;
      u_clear.base = 0;
      u_clear.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->clear = u_clear.real;
      offset += sizeof(this->clear);
     return offset;
    }

    const char * getType(){ return "sub8_msgs/Alarm"; };
    const char * getMD5(){ return "83531a758ff3e6e184d3cd8d7f8f9a8f"; };

  };

}
#endif