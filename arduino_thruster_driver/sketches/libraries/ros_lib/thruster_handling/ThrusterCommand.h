#ifndef _ROS_thruster_handling_ThrusterCommand_h
#define _ROS_thruster_handling_ThrusterCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thruster_handling
{

  class ThrusterCommand : public ros::Msg
  {
    public:
      float force;

    ThrusterCommand():
      force(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->force);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->force));
     return offset;
    }

    const char * getType(){ return "thruster_handling/ThrusterCommand"; };
    const char * getMD5(){ return "e18a51329659ac6263f87aaf9a01fe62"; };

  };

}
#endif