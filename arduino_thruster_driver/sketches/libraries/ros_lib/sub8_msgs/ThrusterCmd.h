#ifndef _ROS_sub8_msgs_ThrusterCmd_h
#define _ROS_sub8_msgs_ThrusterCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sub8_msgs
{

  class ThrusterCmd : public ros::Msg
  {
    public:
      const char* name;
      float thrust;

    ThrusterCmd():
      name(""),
      thrust(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        float real;
        uint32_t base;
      } u_thrust;
      u_thrust.real = this->thrust;
      *(outbuffer + offset + 0) = (u_thrust.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thrust.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thrust.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thrust);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        float real;
        uint32_t base;
      } u_thrust;
      u_thrust.base = 0;
      u_thrust.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_thrust.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_thrust.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->thrust = u_thrust.real;
      offset += sizeof(this->thrust);
     return offset;
    }

    const char * getType(){ return "sub8_msgs/ThrusterCmd"; };
    const char * getMD5(){ return "41fab1b3f6d81d716a13d50bf5e70867"; };

  };

}
#endif