#include <ras_group8_motor_controller/PIDController.hpp>

namespace ras_group8_motor_controller
{

PIDController::PIDController(double gainP, double gainI, double gainD,
                             double outMin, double outMax)
  : gainP_(gainP), gainI_(gainI), gainD_(gainD),
    outMin_(outMin), outMax_(outMax)
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
  derivative = (error - errorPrev_) / dt;
  
  output = gainP_ * error + gainI_ * integral_ + gainD_ * derivative;
  
  /* Fit the output signal between outMax and outMin */
  if (output > outMax_) {
    output = outMax_;
  } else if (output < outMin_) {
    output = outMin_;
  }
  
  errorPrev_ = error;
  
  return output;
}

void PIDController::updateParams(double gainP, double gainI, double gainD,
                                 double outMin, double outMax)
{
  gainP_ = gainP;
  gainI_ = gainI;
  gainD_ = gainD;
  outMin_ = outMin;
  outMax_ = outMax;
}

void PIDController::reset()
{
  integral_ = 0.0;
}

} /* namespace */