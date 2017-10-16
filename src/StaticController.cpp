#include <ras_group8_motor_controller/StaticController.hpp>

namespace ras_group8_motor_controller
{

StaticController::StaticController()
{
}

StaticController::~StaticController()
{
}

double StaticController::update(double value, double target, double dt)
{
  return target;
}

void StaticController::reset()
{
}


} /* namespace */