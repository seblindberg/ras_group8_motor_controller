#pragma once

//#include <ros/ros.h>

namespace ras_group8_motor_controller
{

class PIDController
{
public:
  PIDController(double gainP  = 1.0,
                double gainI  = 0,
                double gainD  = 0,
                double outMin = -1.0,
                double outMax = 1.0);

  virtual ~PIDController();
  
  double update(double value, double target, double dt);
  
  void updateParams(double gainP, double gainI, double gainD, double outMin,
                    double outMax);
  
  void reset();
  
  double getIntegral()
  {
    return integral_;
  }
  
private:
  double gainP_;
  double gainI_;
  double gainD_;
  double outMax_;
  double outMin_;
  
  double errorPrev_;
  double integral_;
};

}
