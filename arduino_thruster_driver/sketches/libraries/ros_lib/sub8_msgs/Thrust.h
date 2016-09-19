#ifndef _ROS_sub8_msgs_Thrust_h
#define _ROS_sub8_msgs_Thrust_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sub8_msgs/ThrusterCmd.h"

namespace sub8_msgs
{

  class Thrust : public ros::Msg
  {
    public:
      uint32_t thruster_commands_length;
      sub8_msgs::ThrusterCmd st_thruster_commands;
      sub8_msgs::ThrusterCmd * thruster_commands;

    Thrust():
      thruster_commands_length(0), thruster_commands(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->thruster_commands_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->thruster_commands_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->thruster_commands_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->thruster_commands_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thruster_commands_length);
      for( uint32_t i = 0; i < thruster_commands_length; i++){
      offset += this->thruster_commands[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t thruster_commands_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      thruster_commands_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      thruster_commands_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      thruster_commands_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->thruster_commands_length);
      if(thruster_commands_lengthT > thruster_commands_length)
        this->thruster_commands = (sub8_msgs::ThrusterCmd*)realloc(this->thruster_commands, thruster_commands_lengthT * sizeof(sub8_msgs::ThrusterCmd));
      thruster_commands_length = thruster_commands_lengthT;
      for( uint32_t i = 0; i < thruster_commands_length; i++){
      offset += this->st_thruster_commands.deserialize(inbuffer + offset);
        memcpy( &(this->thruster_commands[i]), &(this->st_thruster_commands), sizeof(sub8_msgs::ThrusterCmd));
      }
     return offset;
    }

    const char * getType(){ return "sub8_msgs/Thrust"; };
    const char * getMD5(){ return "0719e5bc2c53b7fce2f0fa0fcfe82615"; };

  };

}
#endif