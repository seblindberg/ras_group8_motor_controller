#include <ras_group8_motor_controller/PIDController.hpp>

namespace ras_group8_motor_controller
{

PIDController::PIDController(double gain_p, double gain_i, double gain_d,
                             double out_min, double out_max)
  : gain_p_(gain_p), gain_i_(gain_i), gain_d_(gain_d),
    out_min_(out_min), out_max_(out_max)
{
  integral_ = 0;
}

PIDController::~PIDController()
{
}

double PIDController::update(double value, double target, double dt)
{
  double error;
  double derivative;
  double output;
    
  error = target - value;
  integral_ += error * dt;
  derivative = (error - error_prev_) / dt;
  
  output = gain_p_ * error + gain_i_ * integral_ + gain_d_ * derivative;
  
  //ROS_INFO("e = %f, i = %f, d = %f", error, integral_, derivative);
  
  /* Fit the output signal between out_max and out_min */
  if (output > out_max_) {
    output = out_max_;
  } else if (output < out_min_) {
    output = out_min_;
  }
  
  error_prev_ = error;
  
  return output;
}

void PIDController::reset()
{
  ROS_INFO("Clearing the integral error");
  integral_   = 0.0;
  error_prev_ = 0.0;
}

/* Load parameters from the parameter server and setup a new PIDController.
 */
PIDController PIDController::load(ros::NodeHandle& n)
{
  double gain_p;
  double gain_i;
  double gain_d;
  double out_min;
  double out_max;
  
  if (!n.getParam("gain/p", gain_p))
    exit(-1);
  ROS_INFO("P: gain_p = %f", gain_p);
  
  if (!n.getParam("gain/i", gain_i))
    exit(-1);
  ROS_INFO("P: gain_i = %f", gain_i);
  
  if (!n.getParam("gain/d", gain_d))
    exit(-1);
  ROS_INFO("P: gain_d = %f", gain_d);
  
  if (!n.getParam("output_min", out_min))
    exit(-1);
  ROS_INFO("P: out_min = %f", out_min);
  
  if (!n.getParam("output_max", out_max))
    exit(-1);
  ROS_INFO("P: out_max = %f", out_max);
  
  PIDController object(gain_p, gain_i, gain_d, out_min, out_max);
  
  return object;
}

} /* namespace */