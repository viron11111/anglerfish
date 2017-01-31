#ifndef _ROS_mag_calibration_mag_values_h
#define _ROS_mag_calibration_mag_values_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mag_calibration
{

  class mag_values : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float mag_pre_comp_x;
      float mag_pre_comp_y;
      float mag_pre_comp_z;
      float comp_roll;
      float comp_pitch;
      float comp_yaw;

    mag_values():
      header(),
      mag_pre_comp_x(0),
      mag_pre_comp_y(0),
      mag_pre_comp_z(0),
      comp_roll(0),
      comp_pitch(0),
      comp_yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_mag_pre_comp_x;
      u_mag_pre_comp_x.real = this->mag_pre_comp_x;
      *(outbuffer + offset + 0) = (u_mag_pre_comp_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_pre_comp_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mag_pre_comp_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mag_pre_comp_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mag_pre_comp_x);
      union {
        float real;
        uint32_t base;
      } u_mag_pre_comp_y;
      u_mag_pre_comp_y.real = this->mag_pre_comp_y;
      *(outbuffer + offset + 0) = (u_mag_pre_comp_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_pre_comp_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mag_pre_comp_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mag_pre_comp_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mag_pre_comp_y);
      union {
        float real;
        uint32_t base;
      } u_mag_pre_comp_z;
      u_mag_pre_comp_z.real = this->mag_pre_comp_z;
      *(outbuffer + offset + 0) = (u_mag_pre_comp_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_pre_comp_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mag_pre_comp_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mag_pre_comp_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mag_pre_comp_z);
      union {
        float real;
        uint32_t base;
      } u_comp_roll;
      u_comp_roll.real = this->comp_roll;
      *(outbuffer + offset + 0) = (u_comp_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_comp_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_comp_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_comp_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->comp_roll);
      union {
        float real;
        uint32_t base;
      } u_comp_pitch;
      u_comp_pitch.real = this->comp_pitch;
      *(outbuffer + offset + 0) = (u_comp_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_comp_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_comp_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_comp_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->comp_pitch);
      union {
        float real;
        uint32_t base;
      } u_comp_yaw;
      u_comp_yaw.real = this->comp_yaw;
      *(outbuffer + offset + 0) = (u_comp_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_comp_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_comp_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_comp_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->comp_yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_mag_pre_comp_x;
      u_mag_pre_comp_x.base = 0;
      u_mag_pre_comp_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_pre_comp_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mag_pre_comp_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mag_pre_comp_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mag_pre_comp_x = u_mag_pre_comp_x.real;
      offset += sizeof(this->mag_pre_comp_x);
      union {
        float real;
        uint32_t base;
      } u_mag_pre_comp_y;
      u_mag_pre_comp_y.base = 0;
      u_mag_pre_comp_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_pre_comp_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mag_pre_comp_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mag_pre_comp_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mag_pre_comp_y = u_mag_pre_comp_y.real;
      offset += sizeof(this->mag_pre_comp_y);
      union {
        float real;
        uint32_t base;
      } u_mag_pre_comp_z;
      u_mag_pre_comp_z.base = 0;
      u_mag_pre_comp_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_pre_comp_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mag_pre_comp_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mag_pre_comp_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mag_pre_comp_z = u_mag_pre_comp_z.real;
      offset += sizeof(this->mag_pre_comp_z);
      union {
        float real;
        uint32_t base;
      } u_comp_roll;
      u_comp_roll.base = 0;
      u_comp_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_comp_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_comp_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_comp_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->comp_roll = u_comp_roll.real;
      offset += sizeof(this->comp_roll);
      union {
        float real;
        uint32_t base;
      } u_comp_pitch;
      u_comp_pitch.base = 0;
      u_comp_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_comp_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_comp_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_comp_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->comp_pitch = u_comp_pitch.real;
      offset += sizeof(this->comp_pitch);
      union {
        float real;
        uint32_t base;
      } u_comp_yaw;
      u_comp_yaw.base = 0;
      u_comp_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_comp_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_comp_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_comp_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->comp_yaw = u_comp_yaw.real;
      offset += sizeof(this->comp_yaw);
     return offset;
    }

    const char * getType(){ return "mag_calibration/mag_values"; };
    const char * getMD5(){ return "5d05c6c006fcbdef7b695eedae01bbb2"; };

  };

}
#endif