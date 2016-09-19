#ifndef _ROS_uf_common_PoseTwistStamped_h
#define _ROS_uf_common_PoseTwistStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "uf_common/PoseTwist.h"

namespace uf_common
{

  class PoseTwistStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uf_common::PoseTwist posetwist;

    PoseTwistStamped():
      header(),
      posetwist()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->posetwist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->posetwist.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "uf_common/PoseTwistStamped"; };
    const char * getMD5(){ return "0dd7f91cd3e194d77b03eb96d348c89e"; };

  };

}
#endif