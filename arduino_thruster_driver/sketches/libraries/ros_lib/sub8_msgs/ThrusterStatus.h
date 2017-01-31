#ifndef _ROS_sub8_msgs_ThrusterStatus_h
#define _ROS_sub8_msgs_ThrusterStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sub8_msgs
{

  class ThrusterStatus : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* name;
      float rpm;
      float bus_voltage;
      float bus_current;
      float temperature;
      uint8_t fault;
      uint8_t response_node_id;

    ThrusterStatus():
      header(),
      name(""),
      rpm(0),
      bus_voltage(0),
      bus_current(0),
      temperature(0),
      fault(0),
      response_node_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        float real;
        uint32_t base;
      } u_rpm;
      u_rpm.real = this->rpm;
      *(outbuffer + offset + 0) = (u_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rpm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rpm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rpm);
      union {
        float real;
        uint32_t base;
      } u_bus_voltage;
      u_bus_voltage.real = this->bus_voltage;
      *(outbuffer + offset + 0) = (u_bus_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bus_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bus_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bus_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bus_voltage);
      union {
        float real;
        uint32_t base;
      } u_bus_current;
      u_bus_current.real = this->bus_current;
      *(outbuffer + offset + 0) = (u_bus_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bus_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bus_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bus_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bus_current);
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
      *(outbuffer + offset + 0) = (this->fault >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault);
      *(outbuffer + offset + 0) = (this->response_node_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->response_node_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        float real;
        uint32_t base;
      } u_rpm;
      u_rpm.base = 0;
      u_rpm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rpm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rpm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rpm = u_rpm.real;
      offset += sizeof(this->rpm);
      union {
        float real;
        uint32_t base;
      } u_bus_voltage;
      u_bus_voltage.base = 0;
      u_bus_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bus_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bus_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bus_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bus_voltage = u_bus_voltage.real;
      offset += sizeof(this->bus_voltage);
      union {
        float real;
        uint32_t base;
      } u_bus_current;
      u_bus_current.base = 0;
      u_bus_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bus_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bus_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bus_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bus_current = u_bus_current.real;
      offset += sizeof(this->bus_current);
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
      this->fault =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fault);
      this->response_node_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->response_node_id);
     return offset;
    }

    const char * getType(){ return "sub8_msgs/ThrusterStatus"; };
    const char * getMD5(){ return "fab3c3b6fba32d4881e692bfc8189258"; };

  };

}
#endif