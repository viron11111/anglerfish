#ifndef _ROS_sub8_msgs_Path_h
#define _ROS_sub8_msgs_Path_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sub8_msgs/PathPoint.h"

namespace sub8_msgs
{

  class Path : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t path_length;
      sub8_msgs::PathPoint st_path;
      sub8_msgs::PathPoint * path;

    Path():
      header(),
      path_length(0), path(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->path_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_length);
      for( uint32_t i = 0; i < path_length; i++){
      offset += this->path[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t path_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->path_length);
      if(path_lengthT > path_length)
        this->path = (sub8_msgs::PathPoint*)realloc(this->path, path_lengthT * sizeof(sub8_msgs::PathPoint));
      path_length = path_lengthT;
      for( uint32_t i = 0; i < path_length; i++){
      offset += this->st_path.deserialize(inbuffer + offset);
        memcpy( &(this->path[i]), &(this->st_path), sizeof(sub8_msgs::PathPoint));
      }
     return offset;
    }

    const char * getType(){ return "sub8_msgs/Path"; };
    const char * getMD5(){ return "7fef3d3bcd2c77f767bec58434d81795"; };

  };

}
#endif