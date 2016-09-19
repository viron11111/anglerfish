#ifndef _ROS_uf_common_x_h
#define _ROS_uf_common_x_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uf_common
{

  class x : public ros::Msg
  {
    public:
      int32_t data[8];

    x():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "uf_common/x"; };
    const char * getMD5(){ return "07660a766ba5f3bccde223531a8fc6aa"; };

  };

}
#endif