#ifndef _ROS_SERVICE_TBDetectionSwitch_h
#define _ROS_SERVICE_TBDetectionSwitch_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sub8_msgs
{

static const char TBDETECTIONSWITCH[] = "sub8_msgs/TBDetectionSwitch";

  class TBDetectionSwitchRequest : public ros::Msg
  {
    public:
      bool detection_switch;

    TBDetectionSwitchRequest():
      detection_switch(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_detection_switch;
      u_detection_switch.real = this->detection_switch;
      *(outbuffer + offset + 0) = (u_detection_switch.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->detection_switch);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_detection_switch;
      u_detection_switch.base = 0;
      u_detection_switch.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->detection_switch = u_detection_switch.real;
      offset += sizeof(this->detection_switch);
     return offset;
    }

    const char * getType(){ return TBDETECTIONSWITCH; };
    const char * getMD5(){ return "8e86b6650b8cf77ce464882cdfcb04d6"; };

  };

  class TBDetectionSwitchResponse : public ros::Msg
  {
    public:
      bool success;

    TBDetectionSwitchResponse():
      success(0)
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
     return offset;
    }

    const char * getType(){ return TBDETECTIONSWITCH; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class TBDetectionSwitch {
    public:
    typedef TBDetectionSwitchRequest Request;
    typedef TBDetectionSwitchResponse Response;
  };

}
#endif
