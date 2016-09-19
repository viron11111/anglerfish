#ifndef _ROS_SERVICE_SearchPose_h
#define _ROS_SERVICE_SearchPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace sub8_msgs
{

static const char SEARCHPOSE[] = "sub8_msgs/SearchPose";

  class SearchPoseRequest : public ros::Msg
  {
    public:
      geometry_msgs::Pose intial_position;
      float search_radius;
      bool reset_search;

    SearchPoseRequest():
      intial_position(),
      search_radius(0),
      reset_search(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->intial_position.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_search_radius;
      u_search_radius.real = this->search_radius;
      *(outbuffer + offset + 0) = (u_search_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_search_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_search_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_search_radius.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->search_radius);
      union {
        bool real;
        uint8_t base;
      } u_reset_search;
      u_reset_search.real = this->reset_search;
      *(outbuffer + offset + 0) = (u_reset_search.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reset_search);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->intial_position.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_search_radius;
      u_search_radius.base = 0;
      u_search_radius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_search_radius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_search_radius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_search_radius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->search_radius = u_search_radius.real;
      offset += sizeof(this->search_radius);
      union {
        bool real;
        uint8_t base;
      } u_reset_search;
      u_reset_search.base = 0;
      u_reset_search.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reset_search = u_reset_search.real;
      offset += sizeof(this->reset_search);
     return offset;
    }

    const char * getType(){ return SEARCHPOSE; };
    const char * getMD5(){ return "7111775abdd3022bda24b87eb5e56020"; };

  };

  class SearchPoseResponse : public ros::Msg
  {
    public:
      float area;
      bool found;
      geometry_msgs::Pose target_pose;

    SearchPoseResponse():
      area(0),
      found(0),
      target_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_area;
      u_area.real = this->area;
      *(outbuffer + offset + 0) = (u_area.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_area.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_area.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_area.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->area);
      union {
        bool real;
        uint8_t base;
      } u_found;
      u_found.real = this->found;
      *(outbuffer + offset + 0) = (u_found.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->found);
      offset += this->target_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_area;
      u_area.base = 0;
      u_area.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_area.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_area.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_area.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->area = u_area.real;
      offset += sizeof(this->area);
      union {
        bool real;
        uint8_t base;
      } u_found;
      u_found.base = 0;
      u_found.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->found = u_found.real;
      offset += sizeof(this->found);
      offset += this->target_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SEARCHPOSE; };
    const char * getMD5(){ return "1f461bca41d5850bb48bb5311310bc94"; };

  };

  class SearchPose {
    public:
    typedef SearchPoseRequest Request;
    typedef SearchPoseResponse Response;
  };

}
#endif
