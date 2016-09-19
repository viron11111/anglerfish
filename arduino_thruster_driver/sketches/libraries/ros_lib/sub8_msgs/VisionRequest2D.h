#ifndef _ROS_SERVICE_VisionRequest2D_h
#define _ROS_SERVICE_VisionRequest2D_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/CameraInfo.h"

namespace sub8_msgs
{

static const char VISIONREQUEST2D[] = "sub8_msgs/VisionRequest2D";

  class VisionRequest2DRequest : public ros::Msg
  {
    public:
      const char* target_name;

    VisionRequest2DRequest():
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

    const char * getType(){ return VISIONREQUEST2D; };
    const char * getMD5(){ return "2008933b3c7227647cbe00c74682331a"; };

  };

  class VisionRequest2DResponse : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Pose2D pose;
      int16_t max_x;
      int16_t max_y;
      sensor_msgs::CameraInfo camera_info;
      bool found;

    VisionRequest2DResponse():
      header(),
      pose(),
      max_x(0),
      max_y(0),
      camera_info(),
      found(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_max_x;
      u_max_x.real = this->max_x;
      *(outbuffer + offset + 0) = (u_max_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_x.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->max_x);
      union {
        int16_t real;
        uint16_t base;
      } u_max_y;
      u_max_y.real = this->max_y;
      *(outbuffer + offset + 0) = (u_max_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_y.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->max_y);
      offset += this->camera_info.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_max_x;
      u_max_x.base = 0;
      u_max_x.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_x.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->max_x = u_max_x.real;
      offset += sizeof(this->max_x);
      union {
        int16_t real;
        uint16_t base;
      } u_max_y;
      u_max_y.base = 0;
      u_max_y.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_y.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->max_y = u_max_y.real;
      offset += sizeof(this->max_y);
      offset += this->camera_info.deserialize(inbuffer + offset);
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

    const char * getType(){ return VISIONREQUEST2D; };
    const char * getMD5(){ return "a45fcbec2f45b3f7462dcf21fa1bc1b2"; };

  };

  class VisionRequest2D {
    public:
    typedef VisionRequest2DRequest Request;
    typedef VisionRequest2DResponse Response;
  };

}
#endif
