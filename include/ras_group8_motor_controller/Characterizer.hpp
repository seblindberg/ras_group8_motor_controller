#pragma once

#include <ros/ros.h>
#include <string.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>

namespace ras_group8_motor_controller
{

class Characterizer
{
public:

  Characterizer<class K>(ros::NodeHandle &node_handle,
                         const std::string& wheel_encoder_topic,
                         const std::string& motor_topic);
  
  virtual ~Characterizer();

private:
  void wheelEncoderCallback(const phidgets::motor_encoder& msg);

  /* Node handle
   */
  ros::NodeHandle& node_handle_;

  /* Subscribers
   */
  ros::Subscriber wheel_encoder_subscriber_;

  /* Publishers
   */
  ros::Publisher motor_publisher_;

  /* Parameters
   */
  double wheel_rev_per_meter_;
  double encoder_tics_per_revolution_;

  /* Variables */
  ros::Time last_update_;
  phidgets::motor_encoder encoder_msg_prev_;
};

}
