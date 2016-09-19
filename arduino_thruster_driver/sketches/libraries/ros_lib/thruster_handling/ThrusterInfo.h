#ifndef _ROS_thruster_handling_ThrusterInfo_h
#define _ROS_thruster_handling_ThrusterInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace thruster_handling
{

  class ThrusterInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* id;
      ros::Duration lifetime;
      bool active;
      geometry_msgs::Point position;
      geometry_msgs::Vector3 direction;
      float min_force;
      float max_force;
      geometry_msgs::Vector3 torque_per_force;
      const char* extra;

    ThrusterInfo():
      header(),
      id(""),
      lifetime(),
      active(0),
      position(),
      direction(),
      min_force(0),
      max_force(0),
      torque_per_force(),
      extra("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_id = strlen(this->id);
      memcpy(outbuffer + offset, &length_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      *(outbuffer + offset + 0) = (this->lifetime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lifetime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lifetime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lifetime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lifetime.sec);
      *(outbuffer + offset + 0) = (this->lifetime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lifetime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lifetime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lifetime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lifetime.nsec);
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.real = this->active;
      *(outbuffer + offset + 0) = (u_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->active);
      offset += this->position.serialize(outbuffer + offset);
      offset += this->direction.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_force);
      offset += this->torque_per_force.serialize(outbuffer + offset);
      uint32_t length_extra = strlen(this->extra);
      memcpy(outbuffer + offset, &length_extra, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->extra, length_extra);
      offset += length_extra;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_id;
      memcpy(&length_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      this->lifetime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->lifetime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->lifetime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->lifetime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->lifetime.sec);
      this->lifetime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->lifetime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->lifetime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->lifetime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->lifetime.nsec);
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.base = 0;
      u_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->active = u_active.real;
      offset += sizeof(this->active);
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->direction.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_force));
      offset += this->torque_per_force.deserialize(inbuffer + offset);
      uint32_t length_extra;
      memcpy(&length_extra, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_extra; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_extra-1]=0;
      this->extra = (char *)(inbuffer + offset-1);
      offset += length_extra;
     return offset;
    }

    const char * getType(){ return "thruster_handling/ThrusterInfo"; };
    const char * getMD5(){ return "0290d4aa7b053f85dfadb0eae848d801"; };

  };

}
#endif