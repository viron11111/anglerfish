#ifndef _ROS_sub8_msgs_PathPoint_h
#define _ROS_sub8_msgs_PathPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace sub8_msgs
{

  class PathPoint : public ros::Msg
  {
    public:
      geometry_msgs::Point position;
      float yaw;

    PathPoint():
      position(),
      yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
     return offset;
    }

    const char * getType(){ return "sub8_msgs/PathPoint"; };
    const char * getMD5(){ return "4cc04ac3ba6c1f0f2caa721fb8f71842"; };

  };

}
#endif