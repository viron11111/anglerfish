#ifndef _ROS_SERVICE_UpdateThrusterLayout_h
#define _ROS_SERVICE_UpdateThrusterLayout_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sub8_msgs
{

static const char UPDATETHRUSTERLAYOUT[] = "sub8_msgs/UpdateThrusterLayout";

  class UpdateThrusterLayoutRequest : public ros::Msg
  {
    public:
      uint32_t dropped_thrusters_length;
      char* st_dropped_thrusters;
      char* * dropped_thrusters;

    UpdateThrusterLayoutRequest():
      dropped_thrusters_length(0), dropped_thrusters(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dropped_thrusters_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dropped_thrusters_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dropped_thrusters_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dropped_thrusters_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dropped_thrusters_length);
      for( uint32_t i = 0; i < dropped_thrusters_length; i++){
      uint32_t length_dropped_thrustersi = strlen(this->dropped_thrusters[i]);
      memcpy(outbuffer + offset, &length_dropped_thrustersi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->dropped_thrusters[i], length_dropped_thrustersi);
      offset += length_dropped_thrustersi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t dropped_thrusters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dropped_thrusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dropped_thrusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dropped_thrusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dropped_thrusters_length);
      if(dropped_thrusters_lengthT > dropped_thrusters_length)
        this->dropped_thrusters = (char**)realloc(this->dropped_thrusters, dropped_thrusters_lengthT * sizeof(char*));
      dropped_thrusters_length = dropped_thrusters_lengthT;
      for( uint32_t i = 0; i < dropped_thrusters_length; i++){
      uint32_t length_st_dropped_thrusters;
      memcpy(&length_st_dropped_thrusters, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_dropped_thrusters; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_dropped_thrusters-1]=0;
      this->st_dropped_thrusters = (char *)(inbuffer + offset-1);
      offset += length_st_dropped_thrusters;
        memcpy( &(this->dropped_thrusters[i]), &(this->st_dropped_thrusters), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return UPDATETHRUSTERLAYOUT; };
    const char * getMD5(){ return "b67ed41009ef6b2c76bf4e56fcd5bd49"; };

  };

  class UpdateThrusterLayoutResponse : public ros::Msg
  {
    public:

    UpdateThrusterLayoutResponse()
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

    const char * getType(){ return UPDATETHRUSTERLAYOUT; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class UpdateThrusterLayout {
    public:
    typedef UpdateThrusterLayoutRequest Request;
    typedef UpdateThrusterLayoutResponse Response;
  };

}
#endif
