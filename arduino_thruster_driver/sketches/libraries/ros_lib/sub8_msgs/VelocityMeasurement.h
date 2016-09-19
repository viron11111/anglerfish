#ifndef _ROS_sub8_msgs_VelocityMeasurement_h
#define _ROS_sub8_msgs_VelocityMeasurement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace sub8_msgs
{

  class VelocityMeasurement : public ros::Msg
  {
    public:
      geometry_msgs::Vector3 direction;
      float velocity;
      float correlation;

    VelocityMeasurement():
      direction(),
      velocity(0),
      correlation(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->direction.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->correlation);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->direction.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->correlation));
     return offset;
    }

    const char * getType(){ return "sub8_msgs/VelocityMeasurement"; };
    const char * getMD5(){ return "91bd87f5dd3ca2deb6b495a94d789240"; };

  };

}
#endif