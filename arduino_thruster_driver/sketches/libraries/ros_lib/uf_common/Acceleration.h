#ifndef _ROS_uf_common_Acceleration_h
#define _ROS_uf_common_Acceleration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace uf_common
{

  class Acceleration : public ros::Msg
  {
    public:
      geometry_msgs::Vector3 linear;
      geometry_msgs::Vector3 angular;

    Acceleration():
      linear(),
      angular()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->linear.serialize(outbuffer + offset);
      offset += this->angular.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->linear.deserialize(inbuffer + offset);
      offset += this->angular.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "uf_common/Acceleration"; };
    const char * getMD5(){ return "9f195f881246fdfa2798d1d3eebca84a"; };

  };

}
#endif