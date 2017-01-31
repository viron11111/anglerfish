#ifndef _ROS_SERVICE_PathPlan_h
#define _ROS_SERVICE_PathPlan_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "sub8_msgs/Path.h"

namespace sub8_msgs
{

static const char PATHPLAN[] = "sub8_msgs/PathPlan";

  class PathPlanRequest : public ros::Msg
  {
    public:
      geometry_msgs::Pose start_state;
      geometry_msgs::Pose goal_state;

    PathPlanRequest():
      start_state(),
      goal_state()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->start_state.serialize(outbuffer + offset);
      offset += this->goal_state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->start_state.deserialize(inbuffer + offset);
      offset += this->goal_state.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return PATHPLAN; };
    const char * getMD5(){ return "597a4877231e20839715f9bf31d50817"; };

  };

  class PathPlanResponse : public ros::Msg
  {
    public:
      bool success;
      sub8_msgs::Path path;

    PathPlanResponse():
      success(0),
      path()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      offset += this->path.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      offset += this->path.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return PATHPLAN; };
    const char * getMD5(){ return "62a84827a3f119d9aa08676abb38a9e2"; };

  };

  class PathPlan {
    public:
    typedef PathPlanRequest Request;
    typedef PathPlanResponse Response;
  };

}
#endif
