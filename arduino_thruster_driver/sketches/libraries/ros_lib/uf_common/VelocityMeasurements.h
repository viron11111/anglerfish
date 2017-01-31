#ifndef _ROS_uf_common_VelocityMeasurements_h
#define _ROS_uf_common_VelocityMeasurements_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "uf_common/VelocityMeasurement.h"

namespace uf_common
{

  class VelocityMeasurements : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t velocity_measurements_length;
      uf_common::VelocityMeasurement st_velocity_measurements;
      uf_common::VelocityMeasurement * velocity_measurements;

    VelocityMeasurements():
      header(),
      velocity_measurements_length(0), velocity_measurements(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->velocity_measurements_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_measurements_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity_measurements_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity_measurements_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_measurements_length);
      for( uint32_t i = 0; i < velocity_measurements_length; i++){
      offset += this->velocity_measurements[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t velocity_measurements_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocity_measurements_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocity_measurements_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocity_measurements_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocity_measurements_length);
      if(velocity_measurements_lengthT > velocity_measurements_length)
        this->velocity_measurements = (uf_common::VelocityMeasurement*)realloc(this->velocity_measurements, velocity_measurements_lengthT * sizeof(uf_common::VelocityMeasurement));
      velocity_measurements_length = velocity_measurements_lengthT;
      for( uint32_t i = 0; i < velocity_measurements_length; i++){
      offset += this->st_velocity_measurements.deserialize(inbuffer + offset);
        memcpy( &(this->velocity_measurements[i]), &(this->st_velocity_measurements), sizeof(uf_common::VelocityMeasurement));
      }
     return offset;
    }

    const char * getType(){ return "uf_common/VelocityMeasurements"; };
    const char * getMD5(){ return "5f86da067da8f9f18dda6b6a47259dc8"; };

  };

}
#endif