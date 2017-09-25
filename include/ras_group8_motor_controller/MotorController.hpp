#pragma once

#include <ros/ros.h>
#include <string.h>

namespace ras_group8_motor_controller
{

  class MotorController
  {
  public:
    
    MotorController(ros::NodeHandle &nodeHandle);
    
    virtual ~MotorController();
    
    bool readParameters();
  
  private:
    
    ros::NodeHandle& n_;
    
    std::string subscriberTopic_;
  
  };

}
