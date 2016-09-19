#ifndef _ROS_uf_common_Float64Stamped_h
#define _ROS_uf_common_Float64Stamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace uf_common
{

  class Float64Stamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float data;

    Float64Stamped():
      header(),
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->data));
     return offset;
    }

    const char * getType(){ return "uf_common/Float64Stamped"; };
    const char * getMD5(){ return "e6c99c37e6f9fe98e071d524cc164e65"; };

  };

}
#endif