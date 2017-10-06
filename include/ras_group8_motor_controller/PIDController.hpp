#pragma once

//#include <ros/ros.h>

namespace ras_group8_motor_controller
{

class PIDController
{
public:
  PIDController(double gain_p  = 1.0,
                double gain_i  = 0,
                double gain_d  = 0,
                double out_min = -1.0,
                double out_max = 1.0);

  virtual ~PIDController();
  
  double update(double value, double target, double dt);
  
  void updateParams(double gain_p, double gain_i, double gain_d, double out_min,
                    double out_max);
  
  void reset();
  
  double getIntegral()
  {
    return integral_;
  }
  
private:
  double gain_p_;
  double gain_i_;
  double gain_d_;
  double out_max_;
  double out_min_;
  
  double error_prev_;
  double integral_;
};

}
