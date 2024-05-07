#ifndef _ROS_wholearm_skin_ros_TaxelData_h
#define _ROS_wholearm_skin_ros_TaxelData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace wholearm_skin_ros
{

  class TaxelData : public ros::Msg
  {
    public:
      uint32_t cdc_length;
      typedef uint16_t _cdc_type;
      _cdc_type st_cdc;
      _cdc_type * cdc;
      typedef std_msgs::Header _header_type;
      _header_type header;

    TaxelData():
      cdc_length(0), st_cdc(), cdc(nullptr),
      header()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cdc_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cdc_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cdc_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cdc_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cdc_length);
      for( uint32_t i = 0; i < cdc_length; i++){
      *(outbuffer + offset + 0) = (this->cdc[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cdc[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cdc[i]);
      }
      offset += this->header.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t cdc_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cdc_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cdc_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cdc_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cdc_length);
      if(cdc_lengthT > cdc_length)
        this->cdc = (uint16_t*)realloc(this->cdc, cdc_lengthT * sizeof(uint16_t));
      cdc_length = cdc_lengthT;
      for( uint32_t i = 0; i < cdc_length; i++){
      this->st_cdc =  ((uint16_t) (*(inbuffer + offset)));
      this->st_cdc |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_cdc);
        memcpy( &(this->cdc[i]), &(this->st_cdc), sizeof(uint16_t));
      }
      offset += this->header.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "wholearm_skin_ros/TaxelData"; };
    virtual const char * getMD5() override { return "95e464596080dceb6079e99ff13a7f95"; };

  };

}
#endif
