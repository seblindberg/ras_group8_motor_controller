#include <ras_group8_motor_controller/MotorController.hpp>

namespace ras_group8_motor_controller
{

MotorController::MotorController(ros::NodeHandle &nodeHandle)
  : nodeHandle_(nodeHandle)
{
  if (!reload()) {
    ros::requestShutdown();
  }
  
  /* Setup the reload service
   */
  reloadService_ =
    nodeHandle_.advertiseService("reload", &MotorController::reloadCallback,
                                 this);
  
  ROS_INFO("Successfully launched node.");
}

MotorController::~MotorController()
{
}

void MotorController::wheelEncoderCallback(const phidgets::motor_encoder& msg)
{
}

void MotorController::velocityCallback(const std_msgs::Float32& msg)
{
  ROS_INFO("New velocity: %f", msg.data);
}

template<class M, class T>
void MotorController::updateSubscriber(ros::Subscriber& sub, const std::string newTopic, void(T::*callback)(M))
{
  if (sub) {
    /* Check if the topic has changed */
    if (sub.getTopic().compare(newTopic) == 0) {
      return;
    }
    
    sub.shutdown();
  }
  
  sub = nodeHandle_.subscribe(newTopic, 1, callback, this);
}

/**
 * Reload the parameters used by the node. This will re-subscribe to any topics
 * that are configured via parameters.
 */
bool MotorController::reload()
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    return false;
  }
  
  /* Re-subscribe to topics */
  updateSubscriber(wheelEncoderSubscriber_, wheelEncoderTopic_,
                   &MotorController::wheelEncoderCallback);
                   
  updateSubscriber(velocitySubscriber_, velocityTopic_,
                   &MotorController::velocityCallback);
}

bool MotorController::reloadCallback(std_srvs::Trigger::Request& request,
                                     std_srvs::Trigger::Response& response)
{
  if (reload()) {
    response.success = true;
  } else {
    response.success = false;
    response.message = "Failed to reload node";
  }
  
  return true;
}

bool MotorController::readParameters()
{
  if (!nodeHandle_.getParam("encoder_topic", wheelEncoderTopic_))
    return false;
  ROS_INFO("P: wheelEncoderTopic_ = %s", wheelEncoderTopic_.c_str());
  
  if (!nodeHandle_.getParam("velocity_topic", velocityTopic_))
    return false;
  ROS_INFO("P: velocityTopic_ = %s", velocityTopic_.c_str());
  
  if (!nodeHandle_.getParam("gain/p", gainP_))
    return false;
  ROS_INFO("P: gainP_ = %f", gainP_);
  
  if (!nodeHandle_.getParam("gain/i", gainI_))
    return false;
  ROS_INFO("P: gainI_ = %f", gainI_);
  
  if (!nodeHandle_.getParam("gain/d", gainD_))
    return false;
  ROS_INFO("P: gainD_ = %f", gainD_);
  
  return true;
}

} /* namespace */