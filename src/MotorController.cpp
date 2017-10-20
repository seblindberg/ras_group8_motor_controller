#include <ras_group8_motor_controller/MotorController.hpp>

#include <ras_group8_motor_controller/PIDController.hpp>
#include <ras_group8_motor_controller/StaticController.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <math.h>

namespace ras_group8_motor_controller
{

/* Constructor
 */
template<class Controller>
MotorController<Controller>::MotorController(ros::NodeHandle& node_handle,
                                 Controller& controller,
                                 const std::string& wheel_encoder_topic,
                                 const std::string& velocity_topic,
                                 const std::string& motor_topic,
                                 const std::string& twist_topic,
                                 double rev_per_meter,
                                 double tics_per_rev,
                                 double velocity_expire_timeout,
                                 bool reverse_direction)
  : node_handle_(node_handle),
    controller_(controller),
    velocity_expire_timeout_(velocity_expire_timeout),
    reverse_direction_(reverse_direction),
    meters_per_tics_(1.0 / (rev_per_meter * tics_per_rev))
{
  wheel_encoder_subscriber_ =
    node_handle_.subscribe(wheel_encoder_topic, 1,
                           &MotorController::wheelEncoderCallback, this);
  
  velocity_subscriber_ =
    node_handle_.subscribe(velocity_topic, 1,
                           &MotorController::velocityCallback, this);
                           
  motor_publisher_ =
    node_handle_.advertise<std_msgs::Float32>(motor_topic, 1);
  
  twist_publisher_ =
    node_handle_.advertise<geometry_msgs::TwistStamped>(twist_topic, 1, true);
    
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_STATE
  /* Setup logger outputs here
   * Their enpoints are not configurable
   */
    state_reference_publisher_ =
      node_handle_.advertise<std_msgs::Float32>("reference", 1);
    
    state_input_publisher_ =
      node_handle_.advertise<std_msgs::Float32>("input", 1);
    
    state_output_publisher_ =
      node_handle_.advertise<std_msgs::Float32>("output", 1);
    
  ROS_INFO("Compiled with log output.");
#endif
  
  /* Publish the initial state */
  publishTwist(ros::Time::now(), 0.0);
  
  ROS_INFO("Successfully launched node.");
}

/* Destructor
 */
template<class Controller>
MotorController<Controller>::~MotorController()
{
}

/* Set Target Velocity
 */
template<class Controller>
void MotorController<Controller>::setTargetVelocity(double velocity)
{
  velocity_target_ = velocity;
  velocity_target_expire_time_ = ros::Time::now() + velocity_expire_timeout_;
}

template<class Controller>
void MotorController<Controller>::publishTwist(const ros::Time& now,
                                               double velocity)
{
  static int32_t seq = 0; /* TODO: unsigned? */
  geometry_msgs::TwistStamped twist_msg;
  
  twist_msg.header.stamp   = now;
  twist_msg.header.seq     = seq;
  twist_msg.twist.linear.x = velocity;
  
  twist_publisher_.publish(twist_msg);
  seq += 1;
}

#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_STATE
  /* Optional method that publishes the pid controller state to the three fixed topics reference,
   * input and output.
   */
template<class Controller>
void MotorController<Controller>::publishState(double reference, double input, double output)
{
  std_msgs::Float32 msg;
  
  msg.data = reference;
  state_reference_publisher_.publish(msg);
  
  msg.data = input;
  state_input_publisher_.publish(msg);
  
  msg.data = output;
  state_output_publisher_.publish(msg);
}
#endif

/* Wheel Encoder Callback
 */
template<class Controller>
void MotorController<Controller>::wheelEncoderCallback(const phidgets::motor_encoder& msg)
{
  double velocity;
  double dt;
  std_msgs::Float32 motor_msg;
  geometry_msgs::TwistStamped twist_msg;
  
  /* Calculate delta time */
  dt = (msg.header.stamp - encoder_msg_prev_.header.stamp).toSec();
    
  /* If we don't seem to have missed any messages */
  if (0 < dt && dt < 0.2) { /* TODO: Do not hard-code this value */
    /* Calculate wheel velocity */
    velocity = (double)(msg.count - encoder_msg_prev_.count) /
                 dt * meters_per_tics_;
                
    /* Check that the set velocity has not expired */
    if (velocity_target_expire_time_ < msg.header.stamp) {
      ROS_INFO("No new velocity setting for a while. Setting to zero.");
      velocity_target_ = 0.0;
    }
    
    if (velocity_target_ == 0.0) {
      motor_msg.data = 0.0;
    } else {
      /* Update controller */
      if (reverse_direction_) {
        motor_msg.data =
          controller_.update(velocity, -velocity_target_, dt);
      } else {
        motor_msg.data =
          controller_.update(velocity,  velocity_target_, dt);
      }
    }
    
    if (reverse_direction_) {
      velocity = -velocity;
    }
    
    /* Set new motor value */
    motor_publisher_.publish(motor_msg);
    
    publishTwist(msg.header.stamp, velocity);
    
#if RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_STATE
    /* Publish the internal PID state */
    publishState(velocity_target_, velocity, motor_msg.data);
#endif
  }
    
  /* Store the current message */
  std::memcpy(&encoder_msg_prev_, &msg, sizeof(phidgets::motor_encoder));
  
  velocity_prev_ = velocity;
}

/* Veclocity Callback
 */
template<class Controller>
void MotorController<Controller>::velocityCallback(const std_msgs::Float32::ConstPtr& ptr)
{
  std_msgs::Float32 msg = *ptr;
  
  //ROS_INFO("New velocity: %f [m/s]", msg.data);
  /* Store the expiration time of the velocity */
  setTargetVelocity(msg.data);
  /* Convert from linear velocity (m/s) to wheel velocity (rev/s) */
  //setTargetVelocity(msg.data * wheel_rev_per_meter_);
}

/* Shutdown
 * Shutdown the motor controller by setting the velocity to 0.
 */
template<class Controller>
void MotorController<Controller>::shutdown()
{
  std_msgs::Float32 motor_msg;
  motor_msg.data = 0.0;
  
  /* Set 0 velocity */
  motor_publisher_.publish(motor_msg);
}

/* Load
 */
template<class Controller>
MotorController<Controller>
  MotorController<Controller>::load(ros::NodeHandle &n,
                                    Controller& controller)
{
  double velocity_expire_timeout;
  double wheel_radius;
  
  std::string motor_topic;
  std::string twist_topic("twist");
  std::string wheel_encoder_topic;
  std::string velocity_topic;
  
  double encoder_tics_per_revolution;
  bool reverse_direction;
  double wheel_rev_per_meter;
  
  /* Get required parameters
   */
  if (!n.getParam("motor_topic", motor_topic))
    exit(-1);
  ROS_INFO("P: motor_topic_ = %s", motor_topic.c_str());
  
  if (!n.getParam("encoder_topic", wheel_encoder_topic))
    exit(-1);
  ROS_INFO("P: wheel_encoder_topic_ = %s", wheel_encoder_topic.c_str());
  
  if (!n.getParam("velocity_topic", velocity_topic))
    exit(-1);
  ROS_INFO("P: velocity_topic_ = %s", velocity_topic.c_str());
  
  if (!n.getParam("wheel_encoder_tics_per_rev", encoder_tics_per_revolution))
    exit(-1);
  ROS_INFO("P: encoder_tics_per_revolution_ = %f", encoder_tics_per_revolution);
  
  if (!n.getParam("/platform/wheel_radius", wheel_radius))
    exit(-1);
  ROS_INFO("P: wheel_radius = %f", wheel_radius);
  
  if (!n.getParam("reverse_direction", reverse_direction))
    exit(-1);
  ROS_INFO("P: reverse_direction_ = %u", reverse_direction);
  
  /* Get optional parameters
   */
  twist_topic = n.param("twist_topic", twist_topic);
  ROS_INFO("P: twist_topic = %s", twist_topic.c_str());
  
  velocity_expire_timeout = n.param("velocity_timeout", 0.5);
  ROS_INFO("P: velocity_expire_timeout = %f", velocity_expire_timeout);
  
  /* Calculate wheel_rev_per_meter_ */
  wheel_rev_per_meter = 1.0 / (wheel_radius * 2 * M_PI);
    
  MotorController<Controller> object(n,
                              controller,
                              wheel_encoder_topic,
                              velocity_topic,
                              motor_topic,
                              twist_topic,
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