#ifndef _ROS_ms5837_ms5837_h
#define _ROS_ms5837_ms5837_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ms5837
{

  class ms5837 : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float depth;
      float temperature;
      float ex_pressure;

    ms5837():
      header(),
      depth(0),
      temperature(0),
      ex_pressure(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_depth;
      u_depth.real = this->depth;
      *(outbuffer + offset + 0) = (u_depth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth);
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      union {
        float real;
        uint32_t base;
      } u_ex_pressure;
      u_ex_pressure.real = this->ex_pressure;
      *(outbuffer + offset + 0) = (u_ex_pressure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ex_pressure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ex_pressure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ex_pressure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ex_pressure);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_depth;
      u_depth.base = 0;
      u_depth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_depth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_depth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_depth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->depth = u_depth.real;
      offset += sizeof(this->depth);
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      union {
        float real;
        uint32_t base;
      } u_ex_pressure;
      u_ex_pressure.base = 0;
      u_ex_pressure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ex_pressure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ex_pressure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ex_pressure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ex_pressure = u_ex_pressure.real;
      offset += sizeof(this->ex_pressure);
     return offset;
    }

    const char * getType(){ return "ms5837/ms5837"; };
    const char * getMD5(){ return "b5294c23152db5d718ec3962ae5ce437"; };

  };

}
#endif