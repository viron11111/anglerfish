#ifndef _ROS_SERVICE_BMatrix_h
#define _ROS_SERVICE_BMatrix_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sub8_msgs
{

static const char BMATRIX[] = "sub8_msgs/BMatrix";

  class BMatrixRequest : public ros::Msg
  {
    public:

    BMatrixRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return BMATRIX; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class BMatrixResponse : public ros::Msg
  {
    public:
      uint32_t B_length;
      float st_B;
      float * B;

    BMatrixResponse():
      B_length(0), B(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->B_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->B_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->B_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->B_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->B_length);
      for( uint32_t i = 0; i < B_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->B[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t B_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      B_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      B_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      B_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->B_length);
      if(B_lengthT > B_length)
        this->B = (float*)realloc(this->B, B_lengthT * sizeof(float));
      B_length = B_lengthT;
      for( uint32_t i = 0; i < B_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_B));
        memcpy( &(this->B[i]), &(this->st_B), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return BMATRIX; };
    const char * getMD5(){ return "7bb2fb7c5dcb2a3715f342c547062dd2"; };

  };

  class BMatrix {
    public:
    typedef BMatrixRequest Request;
    typedef BMatrixResponse Response;
  };

}
#endif
