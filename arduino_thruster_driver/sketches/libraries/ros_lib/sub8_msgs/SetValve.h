#ifndef _ROS_SERVICE_SetValve_h
#define _ROS_SERVICE_SetValve_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sub8_msgs
{

static const char SETVALVE[] = "sub8_msgs/SetValve";

  class SetValveRequest : public ros::Msg
  {
    public:
      const char* actuator;
      bool opened;

    SetValveRequest():
      actuator(""),
      opened(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_actuator = strlen(this->actuator);
      memcpy(outbuffer + offset, &length_actuator, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->actuator, length_actuator);
      offset += length_actuator;
      union {
        bool real;
        uint8_t base;
      } u_opened;
      u_opened.real = this->opened;
      *(outbuffer + offset + 0) = (u_opened.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->opened);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_actuator;
      memcpy(&length_actuator, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_actuator; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_actuator-1]=0;
      this->actuator = (char *)(inbuffer + offset-1);
      offset += length_actuator;
      union {
        bool real;
        uint8_t base;
      } u_opened;
      u_opened.base = 0;
      u_opened.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->opened = u_opened.real;
      offset += sizeof(this->opened);
     return offset;
    }

    const char * getType(){ return SETVALVE; };
    const char * getMD5(){ return "9ee7e6f6ddc8fa76fb0aced058add5c6"; };

  };

  class SetValveResponse : public ros::Msg
  {
    public:
      bool success;

    SetValveResponse():
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

    const char * getType(){ return SETVALVE; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetValve {
    public:
    typedef SetValveRequest Request;
    typedef SetValveResponse Response;
  };

}
#endif
