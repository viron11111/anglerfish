#ifndef _ROS_SERVICE_OcrRequest_h
#define _ROS_SERVICE_OcrRequest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sub8_msgs
{

static const char OCRREQUEST[] = "sub8_msgs/OcrRequest";

  class OcrRequestRequest : public ros::Msg
  {
    public:
      const char* target_name;

    OcrRequestRequest():
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

    const char * getType(){ return OCRREQUEST; };
    const char * getMD5(){ return "2008933b3c7227647cbe00c74682331a"; };

  };

  class OcrRequestResponse : public ros::Msg
  {
    public:
      const char* characters;

    OcrRequestResponse():
      characters("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_characters = strlen(this->characters);
      memcpy(outbuffer + offset, &length_characters, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->characters, length_characters);
      offset += length_characters;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_characters;
      memcpy(&length_characters, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_characters; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_characters-1]=0;
      this->characters = (char *)(inbuffer + offset-1);
      offset += length_characters;
     return offset;
    }

    const char * getType(){ return OCRREQUEST; };
    const char * getMD5(){ return "29b8e46b73a741aea213c2b1c1272c8f"; };

  };

  class OcrRequest {
    public:
    typedef OcrRequestRequest Request;
    typedef OcrRequestResponse Response;
  };

}
#endif
