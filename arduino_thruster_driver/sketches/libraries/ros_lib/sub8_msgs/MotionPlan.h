#ifndef _ROS_SERVICE_MotionPlan_h
#define _ROS_SERVICE_MotionPlan_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sub8_msgs/Waypoint.h"
#include "sub8_msgs/Trajectory.h"

namespace sub8_msgs
{

static const char MOTIONPLAN[] = "sub8_msgs/MotionPlan";

  class MotionPlanRequest : public ros::Msg
  {
    public:
      sub8_msgs::Waypoint start_state;
      sub8_msgs::Waypoint goal_state;

    MotionPlanRequest():
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

    const char * getType(){ return MOTIONPLAN; };
    const char * getMD5(){ return "c7123b19d0a028a5fd93fd3829b41412"; };

  };

  class MotionPlanResponse : public ros::Msg
  {
    public:
      bool success;
      sub8_msgs::Trajectory trajectory;

    MotionPlanResponse():
      success(0),
      trajectory()
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
      offset += this->trajectory.serialize(outbuffer + offset);
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
      offset += this->trajectory.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return MOTIONPLAN; };
    const char * getMD5(){ return "d407224eb03cf0cc6fccbb428dbbea67"; };

  };

  class MotionPlan {
    public:
    typedef MotionPlanRequest Request;
    typedef MotionPlanResponse Response;
  };

}
#endif
