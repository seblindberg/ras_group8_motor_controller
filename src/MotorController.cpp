#include <ras_group8_motor_controller/MotorController.hpp>

#include <ras_group8_motor_controller/PIDController.hpp>
#include <ras_group8_motor_controller/StaticController.hpp>

#include <math.h>

namespace ras_group8_motor_controller
{

template<class Controller>
MotorController<Controller>::MotorController(ros::NodeHandle& node_handle,
                                 Controller& controller,
                                 const std::string& wheel_encoder_topic,
                                 const std::string& velocity_topic,
                                 const std::string& motor_topic,
                                 double wheel_rev_per_meter,
                                 double encoder_tics_per_revolution,
                                 double velocity_expire_timeout,
                                 bool reverse_direction)
  : node_handle_(node_handle),
    controller_(controller),
    wheel_rev_per_meter_(wheel_rev_per_meter),
    encoder_tics_per_revolution_(encoder_tics_per_revolution),
    velocity_expire_timeout_(velocity_expire_timeout),
    reverse_direction_(reverse_direction)
{
  wheel_encoder_subscriber_ =
    node_handle_.subscribe(wheel_encoder_topic, 1,
                           &MotorController::wheelEncoderCallback, this);
  
  velocity_subscriber_ =
    node_handle_.subscribe(velocity_topic, 1,
                           &MotorController::velocityCallback, this);
                           
  motor_publisher_ = node_handle_.advertise<std_msgs::Float32>(motor_topic, 1);
    
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  /* Setup logger outputs here
   * Their enpoints are not configurable
   */
    pid_reference_publisher_ =
      node_handle_.advertise<std_msgs::Float32>("reference", 1);
    
    pid_input_publisher_ =
      node_handle_.advertise<std_msgs::Float32>("input", 1);
    
    pid_output_publisher_ =
      node_handle_.advertise<std_msgs::Float32>("output", 1);
    
  ROS_INFO("Compiled with log output.");
#endif
  
  
  ROS_INFO("Successfully launched node.");
}

template<class Controller>
MotorController<Controller>::~MotorController()
{
}

template<class Controller>
void MotorController<Controller>::setTargetVelocity(double velocity)
{
  if (reverse_direction_) {
    velocity_target_ = -velocity;
  } else {
    velocity_target_ = velocity;
  }
}

#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
  /* Optional method that publishes the pid controller state to the three fixed topics reference,
   * input and output.
   */
template<class Controller>
void MotorController<Controller>::publishPidState(double reference, double input, double output)
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

template<class Controller>
void MotorController<Controller>::wheelEncoderCallback(const phidgets::motor_encoder& msg)
{
  double velocity;
  double dt;
  std_msgs::Float32 motor_msg;
  
  /* Calculate delta time */
  dt = (msg.header.stamp - encoder_msg_prev_.header.stamp).toSec();
    
  /* If we don't seem to have missed any messages */
  if (0 < dt && dt < 0.2) { /* TODO: Do not hard-code this value */
    /* Calculate wheel velocity */
    /* TODO: Convert to a multiplication instead of a division */
    velocity = (double)(msg.count - encoder_msg_prev_.count) /
      encoder_tics_per_revolution_ / dt;
                
    /* Check that the set velocity has not expired */
    // if (velocity_target_expire_time_ < msg.header.stamp) {
    //   ROS_INFO("No new velocity setting for a while. Setting to zero.");
    //   velocity_target_ = 0.0;
    // }
  
    if (velocity_target_ == 0.0) {
      motor_msg.data = 0.0;
    } else {
      /* Update controller */
      motor_msg.data =
        controller_.update(velocity, velocity_target_, dt);
    }
    
    /* Set new motor value */
    motor_publisher_.publish(motor_msg);
    
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID
    /* Publish the internal PID state */
    publishPidState(velocity_target_, velocity, motor_msg.data);
#endif
  }
    
  /* Store the current message */
  std::memcpy(&encoder_msg_prev_, &msg, sizeof(phidgets::motor_encoder));
  
  velocity_prev_ = velocity;
}

template<class Controller>
void MotorController<Controller>::velocityCallback(const std_msgs::Float32::ConstPtr& ptr)
{
  std_msgs::Float32 msg = *ptr;
  
  //ROS_INFO("New velocity: %f [m/s]", msg.data);
  /* Store the expiration time of the velocity */
  velocity_target_expire_time_ = ros::Time::now() + velocity_expire_timeout_;
  
  /* Convert from linear velocity (m/s) to wheel velocity (rev/s) */
  setTargetVelocity(msg.data * wheel_rev_per_meter_);
}

/* Shutdown the motor controller by setting the velocity to 0.
 */
template<class Controller>
void MotorController<Controller>::shutdown()
{
  std_msgs::Float32 motor_msg;
  motor_msg.data = 0.0;
  
  motor_publisher_.publish(motor_msg);
}

template<class Controller>
MotorController<Controller>
  MotorController<Controller>::load(ros::NodeHandle &n,
                                    Controller& controller)
{
  double velocity_expire_timeout;
  double wheel_radius;
  
  std::string motor_topic;
  std::string wheel_encoder_topic;
  std::string velocity_topic;
  
  double encoder_tics_per_revolution;
  bool reverse_direction;
  double wheel_rev_per_meter;
  
  
  if (!n.getParam("motor_topic", motor_topic))
    exit(-1);
  ROS_INFO("P: motor_topic_ = %s", motor_topic.c_str());
  
  if (!n.getParam("encoder_topic", wheel_encoder_topic))
    exit(-1);
  ROS_INFO("P: wheel_encoder_topic_ = %s", wheel_encoder_topic.c_str());
  
  if (!n.getParam("velocity_topic", velocity_topic))
    exit(-1);
  ROS_INFO("P: velocity_topic_ = %s", velocity_topic.c_str());
  
  if (!n.getParam("/platform/wheel_encoder_tics_per_rev", encoder_tics_per_revolution))
    exit(-1);
  ROS_INFO("P: encoder_tics_per_revolution_ = %f", encoder_tics_per_revolution);
  
  if (!n.getParam("/platform/wheel_radius", wheel_radius))
    exit(-1);
  ROS_INFO("P: wheel_radius = %f", wheel_radius);
  
  if (!n.getParam("velocity_timeout", velocity_expire_timeout))
    exit(-1);
  ROS_INFO("P: velocity_expire_timeout = %f", velocity_expire_timeout);
  
  if (!n.getParam("reverse_direction", reverse_direction))
    exit(-1);
  ROS_INFO("P: reverse_direction_ = %u", reverse_direction);
    
  /* Calculate wheel_rev_per_meter_ */
  wheel_rev_per_meter = 1.0 / (wheel_radius * 2 * M_PI);
    
  MotorController<Controller> object(n,
                              controller,
                              wheel_encoder_topic,
                              velocity_topic,
                              motor_topic,
                              wheel_rev_per_meter,
                              encoder_tics_per_revolution,
                              velocity_expire_timeout,
                              reverse_direction);
  
  return object;
}

/* Force te compiler to compile the PIDController version of this class.
 */
template class MotorController<PIDController>;
template class MotorController<StaticController>;

} /* namespace */