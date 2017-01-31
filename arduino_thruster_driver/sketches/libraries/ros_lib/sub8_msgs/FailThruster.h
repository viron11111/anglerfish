#ifndef _ROS_SERVICE_FailThruster_h
#define _ROS_SERVICE_FailThruster_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sub8_msgs
{

static const char FAILTHRUSTER[] = "sub8_msgs/FailThruster";

  class FailThrusterRequest : public ros::Msg
  {
    public:
      const char* thruster_name;

    FailThrusterRequest():
      thruster_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_thruster_name = strlen(this->thruster_name);
      memcpy(outbuffer + offset, &length_thruster_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->thruster_name, length_thruster_name);
      offset += length_thruster_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_thruster_name;
      memcpy(&length_thruster_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_thruster_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_thruster_name-1]=0;
      this->thruster_name = (char *)(inbuffer + offset-1);
      offset += length_thruster_name;
     return offset;
    }

    const char * getType(){ return FAILTHRUSTER; };
    const char * getMD5(){ return "5751b33846937709761da534dafa6d35"; };

  };

  class FailThrusterResponse : public ros::Msg
  {
    public:

    FailThrusterResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return FAILTHRUSTER; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class FailThruster {
    public:
    typedef FailThrusterRequest Request;
    typedef FailThrusterResponse Response;
  };

}
#endif
