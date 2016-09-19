#ifndef _ROS_SERVICE_VisionRequest_h
#define _ROS_SERVICE_VisionRequest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"

namespace sub8_msgs
{

static const char VISIONREQUEST[] = "sub8_msgs/VisionRequest";

  class VisionRequestRequest : public ros::Msg
  {
    public:
      const char* target_name;

    VisionRequestRequest():
      target_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_target_name = strlen(this->target_name);
      memcpy(outbuffer + offset, &length_target_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->target_name, length_target_name);
      offset += length_target_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_target_name;
      memcpy(&length_target_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_name-1]=0;
      this->target_name = (char *)(inbuffer + offset-1);
      offset += length_target_name;
     return offset;
    }

    const char * getType(){ return VISIONREQUEST; };
    const char * getMD5(){ return "2008933b3c7227647cbe00c74682331a"; };

  };

  class VisionRequestResponse : public ros::Msg
  {
    public:
      geometry_msgs::PoseStamped pose;
      geometry_msgs::Vector3 covariance_diagonal;
      bool found;

    VisionRequestResponse():
      pose(),
      covariance_diagonal(),
      found(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->covariance_diagonal.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_found;
      u_found.real = this->found;
      *(outbuffer + offset + 0) = (u_found.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->found);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->covariance_diagonal.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_found;
      u_found.base = 0;
      u_found.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->found = u_found.real;
      offset += sizeof(this->found);
     return offset;
    }

    const char * getType(){ return VISIONREQUEST; };
    const char * getMD5(){ return "d818241cae928a4bd838673ab5651be7"; };

  };

  class VisionRequest {
    public:
    typedef VisionRequestRequest Request;
    typedef VisionRequestResponse Response;
  };

}
#endif
