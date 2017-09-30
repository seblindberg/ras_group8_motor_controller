#pragma once

#include <ros/ros.h>
#include <string.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>

namespace ras_group8_motor_controller
{

class MotorController
{
public:
  
  MotorController(ros::NodeHandle &nodeHandle);
  
  virtual ~MotorController();
  
  bool readParameters();
  
  /* Call this method to force the node to reload all of its parameter settings.
   * More commonly this would also be attached to a message callback.
   */
  bool reload();

private:
  void wheelEncoderCallback(const phidgets::motor_encoder& msg);
  
  void velocityCallback(const std_msgs::Float32& msg);
  
  bool reloadCallback(std_srvs::Trigger::Request& request,
                      std_srvs::Trigger::Response& response);
  
  template<class M, class T>
  void updateSubscriber(ros::Subscriber& sub, const std::string newTopic,
                        void(T::*callback)(M));
  
  /* Node handle
   */
  ros::NodeHandle& nodeHandle_;
  
  /* Subscribers
   */
  ros::Subscriber wheelEncoderSubscriber_;
  ros::Subscriber velocitySubscriber_;
  
  /* Publishers
   */
  
  /* Services
   */
  ros::ServiceServer reloadService_;
  
  /* Parameters
   */
  std::string wheelEncoderTopic_;
  std::string velocityTopic_;
  
  double gainP_;
  double gainI_;
  double gainD_;
};

}
