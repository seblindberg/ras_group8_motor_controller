#include <ras_group8_motor_controller/MotorController.hpp>
#include <math.h>

namespace ras_group8_motor_controller
{

MotorController::MotorController(ros::NodeHandle &node_handle)
  : node_handle_(node_handle)
{
  if (!reload()) {
    ros::requestShutdown();
  }
  
  /* Setup the reload service
   */
  reload_service_ =
    node_handle_.advertiseService("reload", &MotorController::reloadCallback,
                                 this);
  
  wheel_encoder_callback_ = &MotorController::wheelEncoderCallback;
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  /* Setup logger outputs here
    *  Their enpoints are not configurable
   */
    pid_reference_publisher_ = node_handle_.advertise<std_msgs::Float32>("reference", 1);
    pid_input_publisher_        = node_handle_.advertise<std_msgs::Float32>("input", 1);
    pid_output_publisher_     = node_handle_.advertise<std_msgs::Float32>("output", 1);
  ROS_INFO("Compiled with log output.");
#endif
  
  
  ROS_INFO("Successfully launched node.");
}

MotorController::~MotorController()
{
}

#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  /* Optional method that publishes the pid controller state to the three fixed topics reference,
   * input and output.
   */
void MotorController::publishPidState(double reference, double input, double output)
{
  std_msgs::Float32 msg;
  
  msg.data = reference;
  pid_reference_publisher_.publish(msg);
  
  msg.data = input;
  pid_input_publisher_.publish(msg);
  
  msg.data = output;
  pid_output_publisher_.publish(msg);
}
#endif

void MotorController::wheelEncoderCallbackOneshot(const phidgets::motor_encoder& msg)
{
  /* Store the current message */
  std::memcpy(&encoder_msg_prev_, &msg, sizeof(phidgets::motor_encoder));
  
  /* Arm the regular callback */
  updateSubscriber(wheel_encoder_subscriber_, wheel_encoder_topic_,
                   &MotorController::wheelEncoderCallback);
}

void MotorController::wheelEncoderCallback(const phidgets::motor_encoder& msg)
{
  double velocity;
  double dt;
  std_msgs::Float32 motor_msg;
  
  /* Calculate delta time */
  dt = (msg.header.stamp - encoder_msg_prev_.header.stamp).toSec();
  
  /* If we don't seem to have missed any messages */
  if (0 > dt && dt < 0.2) { /* TODO: Do not hard-code this value */
    /* Calculate wheel velocity */
    /* TODO: Convert to a multiplication instead of a division */
    velocity = (double)(msg.count - encoder_msg_prev_.count) /
      encoder_tics_per_revolution_ / dt;
                
    /* Check that the set velocity has not expired */
    if (velocity_target_expire_time_ < msg.header.stamp) {
      ROS_INFO("No new velocity setting for a while. Setting to zero.");
      velocity_target_ = 0.0;
    }
                
    /* Update controller */
    motor_msg.data =
      pid_controller_.update(velocity, velocity_target_, dt);
    
    /* Set new motor value */
    motor_publisher_.publish(motor_msg);
    
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
    /* Publish the internal PID state */
    publishPidState(velocity_target_, velocity, motor_msg.data);
#endif
  }
    
  /* Store the current message */
  std::memcpy(&encoder_msg_prev_, &msg, sizeof(phidgets::motor_encoder));
}

void MotorController::velocityCallback(const std_msgs::Float32::ConstPtr& ptr)
{
  std_msgs::Float32 msg = *ptr;
  
  ROS_INFO("New velocity: %f [m/s]", msg.data);
  /* Store the expiration time of the velocity */
  velocity_target_expire_time_ = ros::Time::now() + velocity_expire_timeout_;
  
  /* Convert from linear velocity (m/s) to wheel velocity (rev/s) */
  velocity_target_ = msg.data * wheel_rev_per_meter_;
  
  if (reverse_direction_) {
    velocity_target_ = -velocity_target_;
  }
}

template<class M, class T>
void MotorController::updateSubscriber(ros::Subscriber& sub, const std::string new_topic, void(T::*callback)(M))
{
  if (NULL != sub) {
    /* Check if the topic has changed */
    if (sub.getTopic().compare(new_topic) == 0) {
      return;
    }
    
    sub.shutdown();
  }
  
  sub = node_handle_.subscribe(new_topic, 1, callback, this);
}

template<class M>
void MotorController::updatePublisher(ros::Publisher& pub,
                                      const std::string new_topic)
{
  if (NULL != pub) {
    /* Check if the topic has changed */
    if (pub.getTopic().compare(new_topic) == 0) {
      return;
    }

    pub.shutdown();
  }

  pub = node_handle_.advertise<M>(new_topic, 1);
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
  updateSubscriber(wheel_encoder_subscriber_, wheel_encoder_topic_,
                   wheel_encoder_callback_);
                   
  updateSubscriber(velocity_subscriber_, velocity_topic_,
                   &MotorController::velocityCallback);

  updatePublisher<std_msgs::Float32>(motor_publisher_, motor_topic_);
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
  double gain_p;
  double gain_i;
  double gain_d;
  double out_min;
  double out_max;
  double velocity_expire_timeout;
  double wheel_radius;
  
  if (!node_handle_.getParam("motor_topic", motor_topic_))
    return false;
  ROS_INFO("P: motor_topic_ = %s", motor_topic_.c_str());
  
  if (!node_handle_.getParam("encoder_topic", wheel_encoder_topic_))
    return false;
  ROS_INFO("P: wheel_encoder_topic_ = %s", wheel_encoder_topic_.c_str());
  
  if (!node_handle_.getParam("velocity_topic", velocity_topic_))
    return false;
  ROS_INFO("P: velocity_topic_ = %s", velocity_topic_.c_str());
  
  if (!node_handle_.getParam("gain/p", gain_p))
    return false;
  ROS_INFO("P: gain_p = %f", gain_p);
  
  if (!node_handle_.getParam("gain/i", gain_i))
    return false;
  ROS_INFO("P: gain_i = %f", gain_i);
  
  if (!node_handle_.getParam("gain/d", gain_d))
    return false;
  ROS_INFO("P: gain_d = %f", gain_d);
  
  if (!node_handle_.getParam("output_min", out_min))
    return false;
  ROS_INFO("P: out_min = %f", out_min);
  
  if (!node_handle_.getParam("output_max", out_max))
    return false;
  ROS_INFO("P: out_max = %f", out_max);
  
  if (!node_handle_.getParam("/platform/wheel_encoder_tics_per_rev", encoder_tics_per_revolution_))
    return false;
  ROS_INFO("P: encoder_tics_per_revolution_ = %f", encoder_tics_per_revolution_);
  
  if (!node_handle_.getParam("/platform/wheel_radius", wheel_radius))
    return false;
  ROS_INFO("P: wheel_radius = %f", wheel_radius);
  
  if (!node_handle_.getParam("velocity_timeout", velocity_expire_timeout))
    return false;
  ROS_INFO("P: velocity_expire_timeout = %f", velocity_expire_timeout);
  
  if (!node_handle_.getParam("reverse_direction", reverse_direction_))
    return false;
  ROS_INFO("P: reverse_direction_ = %u", reverse_direction_);
  
  /* Update the PID parameters */
  pid_controller_.updateParams(gain_p, gain_i, gain_d, out_min, out_max);
  pid_controller_.reset();
  
  /* Calculate wheel_rev_per_meter_ */
  wheel_rev_per_meter_ = 1.0 / (wheel_radius * 2 * M_PI);
  
  /* Wrap the expire timeout in a duration */
  velocity_expire_timeout_ = ros::Duration(velocity_expire_timeout);
  
  return true;
}

} /* namespace */