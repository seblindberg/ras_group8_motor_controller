#pragma once

#include <ros/ros.h>

namespace ras_group8_motor_controller
{

class StaticController
{
public:
  StaticController();

  virtual ~StaticController();

  double update(double value, double target, double dt);

  void reset();
};

}
