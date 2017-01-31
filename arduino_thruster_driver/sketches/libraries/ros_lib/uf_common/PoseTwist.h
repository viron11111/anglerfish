#ifndef _ROS_uf_common_PoseTwist_h
#define _ROS_uf_common_PoseTwist_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "uf_common/Acceleration.h"

namespace uf_common
{

  class PoseTwist : public ros::Msg
  {
    public:
      geometry_msgs::Pose pose;
      geometry_msgs::Twist twist;
      uf_common::Acceleration acceleration;

    PoseTwist():
      pose(),
      twist(),
      acceleration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      offset += this->acceleration.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
      offset += this->acceleration.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "uf_common/PoseTwist"; };
    const char * getMD5(){ return "216d253eabd1205fb9f801b844ae56ba"; };

  };

}
#endif