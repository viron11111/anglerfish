#ifndef _ROS_SERVICE_TorpBoardPoseRequest_h
#define _ROS_SERVICE_TorpBoardPoseRequest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"

namespace sub8_msgs
{

static const char TORPBOARDPOSEREQUEST[] = "sub8_msgs/TorpBoardPoseRequest";

  class TorpBoardPoseRequestRequest : public ros::Msg
  {
    public:
      geometry_msgs::PoseStamped pose_stamped;
      float l_proj_mat[12];
      float r_proj_mat[12];
      geometry_msgs::Pose2D l_obs_corners[4];
      geometry_msgs::Pose2D r_obs_corners[4];

    TorpBoardPoseRequestRequest():
      pose_stamped(),
      l_proj_mat(),
      r_proj_mat(),
      l_obs_corners(),
      r_obs_corners()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose_stamped.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 12; i++){
      union {
        float real;
        uint32_t base;
      } u_l_proj_mati;
      u_l_proj_mati.real = this->l_proj_mat[i];
      *(outbuffer + offset + 0) = (u_l_proj_mati.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_proj_mati.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_proj_mati.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_proj_mati.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_proj_mat[i]);
      }
      for( uint32_t i = 0; i < 12; i++){
      union {
        float real;
        uint32_t base;
      } u_r_proj_mati;
      u_r_proj_mati.real = this->r_proj_mat[i];
      *(outbuffer + offset + 0) = (u_r_proj_mati.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_proj_mati.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_proj_mati.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_proj_mati.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_proj_mat[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->l_obs_corners[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->r_obs_corners[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose_stamped.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 12; i++){
      union {
        float real;
        uint32_t base;
      } u_l_proj_mati;
      u_l_proj_mati.base = 0;
      u_l_proj_mati.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_proj_mati.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_proj_mati.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_proj_mati.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_proj_mat[i] = u_l_proj_mati.real;
      offset += sizeof(this->l_proj_mat[i]);
      }
      for( uint32_t i = 0; i < 12; i++){
      union {
        float real;
        uint32_t base;
      } u_r_proj_mati;
      u_r_proj_mati.base = 0;
      u_r_proj_mati.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_proj_mati.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_proj_mati.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_proj_mati.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_proj_mat[i] = u_r_proj_mati.real;
      offset += sizeof(this->r_proj_mat[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->l_obs_corners[i].deserialize(inbuffer + offset);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += this->r_obs_corners[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return TORPBOARDPOSEREQUEST; };
    const char * getMD5(){ return "f6bad5912b8ef954c1eee27ef485fb02"; };

  };

  class TorpBoardPoseRequestResponse : public ros::Msg
  {
    public:
      bool success;

    TorpBoardPoseRequestResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return TORPBOARDPOSEREQUEST; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class TorpBoardPoseRequest {
    public:
    typedef TorpBoardPoseRequestRequest Request;
    typedef TorpBoardPoseRequestResponse Response;
  };

}
#endif
