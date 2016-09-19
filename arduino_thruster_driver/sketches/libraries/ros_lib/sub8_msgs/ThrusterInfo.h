#ifndef _ROS_SERVICE_ThrusterInfo_h
#define _ROS_SERVICE_ThrusterInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sub8_msgs
{

static const char THRUSTERINFO[] = "sub8_msgs/ThrusterInfo";

  class ThrusterInfoRequest : public ros::Msg
  {
    public:
      uint8_t thruster_id;

    ThrusterInfoRequest():
      thruster_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->thruster_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->thruster_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->thruster_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->thruster_id);
     return offset;
    }

    const char * getType(){ return THRUSTERINFO; };
    const char * getMD5(){ return "240fe4c559a653a019a6414b58df2bef"; };

  };

  class ThrusterInfoResponse : public ros::Msg
  {
    public:
      float min_force;
      float max_force;

    ThrusterInfoResponse():
      min_force(0),
      max_force(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_min_force;
      u_min_force.real = this->min_force;
      *(outbuffer + offset + 0) = (u_min_force.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_force.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_force.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_force.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_force);
      union {
        float real;
        uint32_t base;
      } u_max_force;
      u_max_force.real = this->max_force;
      *(outbuffer + offset + 0) = (u_max_force.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_force.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_force.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_force.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_force);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_min_force;
      u_min_force.base = 0;
      u_min_force.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_force.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_force.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_force.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_force = u_min_force.real;
      offset += sizeof(this->min_force);
      union {
        float real;
        uint32_t base;
      } u_max_force;
      u_max_force.base = 0;
      u_max_force.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_force.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_force.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_force.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_force = u_max_force.real;
      offset += sizeof(this->max_force);
     return offset;
    }

    const char * getType(){ return THRUSTERINFO; };
    const char * getMD5(){ return "b5a2d507be94fcf3a3455a55156bb8dd"; };

  };

  class ThrusterInfo {
    public:
    typedef ThrusterInfoRequest Request;
    typedef ThrusterInfoResponse Response;
  };

}
#endif
