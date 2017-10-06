#include <ras_group8_motor_controller/PIDController.hpp>
#include <ros/ros.h>

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
  
  ROS_INFO("e = %f, i = %f, d = %f", error, integral_, derivative);
  
  /* Fit the output signal between out_max and out_min */
  if (output > out_max_) {
    output = out_max_;
  } else if (output < out_min_) {
    output = out_min_;
  }
  
  error_prev_ = error;
  
  return output;
}

void PIDController::updateParams(double gain_p, double gain_i, double gain_d,
                                 double out_min, double out_max)
{
  gain_p_ = gain_p;
  gain_i_ = gain_i;
  gain_d_ = gain_d;
  out_min_ = out_min;
  out_max_ = out_max;
}

void PIDController::reset()
{
  ROS_INFO("Clearing the integral error");
  integral_ = 0.0;
}

} /* namespace */