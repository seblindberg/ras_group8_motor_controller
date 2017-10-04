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
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  /* Setup logger outputs here
    *  Their enpoints are not configurable
   */
    pidReferencePublisher_ = nodeHandle_.advertise<std_msgs::Float32>("reference", 1);
    pidInputPublisher_        = nodeHandle_.advertise<std_msgs::Float32>("input", 1);
    pidOutputPublisher_     = nodeHandle_.advertise<std_msgs::Float32>("output", 1);
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
  pidReferencePublisher_.publish(msg);
  
  msg.data = input;
  pidInputPublisher_.publish(msg);
  
  msg.data = output;
  pidOutputPublisher_.publish(msg);
}
#endif

void MotorController::wheelEncoderCallback(const phidgets::motor_encoder& msg)
{
  double velocity;
  double time = ros::Time::now().toSec();
  double dt;
  std_msgs::Float32 motorMsg;
  
  /* Calculate delta time */
  dt = (msg.header.stamp - encoderMsgPrev_.header.stamp).toSec();
  /* Calculate wheel velocity */
  /* TODO: Convert to a multiplication instead of a division */
  velocity = (msg.count - encoderMsgPrev_.count) /
              encoderTicsPerRevolution_ / dt;
  /* Update controller */
  motorMsg.data =
    pidController_.update(velocity, velocityTarget_, dt);
  /* Set new motor value */
  motorPublisher_.publish(motorMsg);
  /* Store the current message */
  std::memcpy(&encoderMsgPrev_, &msg, sizeof(phidgets::motor_encoder));
  
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  /* .Publish the internal PID state */
  publishPidState(velocityTarget_, velocity, motorMsg.data);
#endif
}

void MotorController::velocityCallback(const std_msgs::Float32::ConstPtr& ptr)
{
  std_msgs::Float32 msg = *ptr;
  
  ROS_INFO("New velocity: %f", msg.data);
  
  velocityTarget_ = msg.data;
}

template<class M, class T>
void MotorController::updateSubscriber(ros::Subscriber& sub, const std::string newTopic, void(T::*callback)(M))
{
  if (NULL != sub) {
    /* Check if the topic has changed */
    if (sub.getTopic().compare(newTopic) == 0) {
      return;
    }
    
    sub.shutdown();
  }
  
  sub = nodeHandle_.subscribe(newTopic, 1, callback, this);
}

template<class M>
void MotorController::updatePublisher(ros::Publisher& pub,
                                      const std::string newTopic)
{
  if (NULL != pub) {
    /* Check if the topic has changed */
    if (pub.getTopic().compare(newTopic) == 0) {
      return;
    }

    pub.shutdown();
  }

  pub = nodeHandle_.advertise<M>(newTopic, 1);
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

  updatePublisher<std_msgs::Float32>(motorPublisher_, motorTopic_);
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
  double gainP;
  double gainI;
  double gainD;
  double outMin;
  double outMax;
  
  if (!nodeHandle_.getParam("motor_topic", motorTopic_))
    return false;
  ROS_INFO("P: motorTopic_ = %s", motorTopic_.c_str());
  
  if (!nodeHandle_.getParam("encoder_topic", wheelEncoderTopic_))
    return false;
  ROS_INFO("P: wheelEncoderTopic_ = %s", wheelEncoderTopic_.c_str());
  
  if (!nodeHandle_.getParam("velocity_topic", velocityTopic_))
    return false;
  ROS_INFO("P: velocityTopic_ = %s", velocityTopic_.c_str());
  
  if (!nodeHandle_.getParam("gain/p", gainP))
    return false;
  ROS_INFO("P: gainP = %f", gainP);
  
  if (!nodeHandle_.getParam("gain/i", gainI))
    return false;
  ROS_INFO("P: gainI = %f", gainI);
  
  if (!nodeHandle_.getParam("gain/d", gainD))
    return false;
  ROS_INFO("P: gainD = %f", gainD);
  
  if (!nodeHandle_.getParam("output_min", outMin))
    return false;
  ROS_INFO("P: outMin = %f", outMin);
  
  if (!nodeHandle_.getParam("output_max", outMax))
    return false;
  ROS_INFO("P: outMax = %f", outMax);
  
  if (!nodeHandle_.getParam("encoder_tics_per_rev", encoderTicsPerRevolution_))
    return false;
  ROS_INFO("P: encoderTicsPerRevolution_ = %f", encoderTicsPerRevolution_);
  
  /* Update the PID parameters */
  pidController_.updateParams(gainP, gainI, gainD, outMin, outMax);
  
  return true;
}

} /* namespace */