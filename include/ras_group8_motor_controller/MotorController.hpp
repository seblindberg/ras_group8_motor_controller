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
  
  MotorController(ros::NodeHandle &node_handle);
  
  virtual ~MotorController();
  
  bool readParameters();
  
  /* Call this method to force the node to reload all of its parameter settings.
   * More commonly this would also be attached to a message callback.
   */
  bool reload();

private:
  void wheelEncoderCallbackOneshot(const phidgets::motor_encoder& msg);
  void wheelEncoderCallback(const phidgets::motor_encoder& msg);
  
  /* Accept the linear velocity as a float in m/s */
  void velocityCallback(const std_msgs::Float32::ConstPtr& msg);
  
  bool reloadCallback(std_srvs::Trigger::Request& request,
                      std_srvs::Trigger::Response& response);
  
  template<class M, class T>
  void updateSubscriber(ros::Subscriber& sub, const std::string new_topic,
                        void(T::*callback)(M));
  
  template<class M>
  void updatePublisher(ros::Publisher& pub, const std::string new_topic);
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  /* Optional method that publishes the pid controller state to the three fixed topics reference,
   * input and output.
   */
  void publishPidState(double reference, double input, double output);
#endif
  
  /* Node handle
   */
  ros::NodeHandle& node_handle_;
  
  /* Subscribers
   */
  ros::Subscriber wheel_encoder_subscriber_;
  ros::Subscriber velocity_subscriber_;
  
  /* Publishers
   */
  ros::Publisher motor_publisher_;
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  ros::Publisher pid_reference_publisher_;
  ros::Publisher pid_input_publisher_;
  ros::Publisher pid_output_publisher_;
#endif
  
  /* Services
   */
  ros::ServiceServer reload_service_;
  
  /* Parameters
   */
  std::string wheel_encoder_topic_;
  std::string velocity_topic_;
  std::string motor_topic_;
  
  double wheel_rev_per_meter_;
  double encoder_tics_per_revolution_;
  ros::Duration velocity_expire_timeout_;
  bool reverse_direction_;
  
  /* Variables */
  PIDController pid_controller_;
  double velocity_target_;
  ros::Time velocity_target_expire_time_;
  phidgets::motor_encoder encoder_msg_prev_;
};

}
