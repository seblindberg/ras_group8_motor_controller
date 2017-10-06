#pragma once

#include <ros/ros.h>
#include <string.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <ras_group8_motor_controller/PIDController.hpp>

#define RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID 1

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
  
  void velocityCallback(const std_msgs::Float32::ConstPtr& msg);
  
  bool reloadCallback(std_srvs::Trigger::Request& request,
                      std_srvs::Trigger::Response& response);
  
  template<class M, class T>
  void updateSubscriber(ros::Subscriber& sub, const std::string newTopic,
                        void(T::*callback)(M));
  
  template<class M>
  void updatePublisher(ros::Publisher& pub, const std::string newTopic);
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  /* Optional method that publishes the pid controller state to the three fixed topics reference,
   * input and output.
   */
  void publishPidState(double reference, double input, double output);
#endif
  
  /* Node handle
   */
  ros::NodeHandle& nodeHandle_;
  
  /* Subscribers
   */
  ros::Subscriber wheelEncoderSubscriber_;
  ros::Subscriber velocitySubscriber_;
  
  /* Publishers
   */
  ros::Publisher motorPublisher_;
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  ros::Publisher pidReferencePublisher_;
  ros::Publisher pidInputPublisher_;
  ros::Publisher pidOutputPublisher_;
#endif
  
  /* Services
   */
  ros::ServiceServer reloadService_;
  
  /* Parameters
   */
  std::string wheelEncoderTopic_;
  std::string velocityTopic_;
  std::string motorTopic_;
  
  double encoderTicsPerRevolution_;
  ros::Duration velocityExpireTimeout_;
  
  /**/
  PIDController pidController_;
  double velocityTarget_;
  ros::Time velocityTargetExpireTime_;
  
  phidgets::motor_encoder encoderMsgPrev_;
};

}
