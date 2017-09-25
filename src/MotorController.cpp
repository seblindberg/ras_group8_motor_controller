#include <ras_group8_motor_controller/MotorController.hpp>
#include <string>

namespace ras_group8_motor_controller
{
  MotorController::MotorController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
  {
    if (!readParameters()) {
      ROS_ERROR("Could not read parameters.");
      ros::requestShutdown();
    }
    
    ROS_INFO("Successfully launched node.");
  }
  
  MotorController::~MotorController()
  {
  }
  
  bool MotorController::readParameters()
  {
    if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) {
      return false;
    }
    
    return true;
  }
}