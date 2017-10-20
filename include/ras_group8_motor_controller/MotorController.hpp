#pragma once

#include <ros/ros.h>
#include <string.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>

#define RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_STATE 1

namespace ras_group8_motor_controller
{

template<class Controller>
class MotorController
{
public:
  MotorController<Controller>(ros::NodeHandle& node_handle,
                  Controller& controller,
                  const std::string& wheel_encoder_topic,
                  const std::string& velocity_topic,
                  const std::string& motor_topic,
                  const std::string& twist_topic,
                  double wheel_rev_per_meter,
                  double encoder_tics_per_revolution,
                  double velocity_expire_timeout,
                  bool reverse_direction);
  
  virtual
    ~MotorController();
  
  void
    setTargetVelocity(double velocity);
  
  double
    velocity()
  {
    return velocity_prev_;
  }
  
  void
    shutdown();
  
  static MotorController<Controller>
    load(ros::NodeHandle &n, Controller& controller);

private:
  void
    wheelEncoderCallback(const phidgets::motor_encoder& msg);
  
  /* Accept the linear velocity as a float in m/s */
  void
    velocityCallback(const std_msgs::Float32::ConstPtr& msg);
  
  void
    publishTwist(const ros::Time& now, double velocity);
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_STATE
  /* Optional method that publishes the pid controller state to the three fixed
   * topics reference, input and output.
   */
  void
    publishState(double reference, double input, double output);
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
  ros::Publisher twist_publisher_;
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_STATE
  ros::Publisher state_reference_publisher_;
  ros::Publisher state_input_publisher_;
  ros::Publisher state_output_publisher_;
#endif
    
  /* Parameters
   */
  const ros::Duration velocity_expire_timeout_;
  const bool reverse_direction_;
  const double meters_per_tics_;
  
  /* Variables */
  Controller& controller_;
  double velocity_target_;
  double velocity_prev_;
  ros::Time velocity_target_expire_time_;
  phidgets::motor_encoder encoder_msg_prev_;
};

}
