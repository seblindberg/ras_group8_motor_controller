#include <ros/ros.h>
#include <ras_group8_motor_controller/MotorController.hpp>
#include <ras_group8_motor_controller/PIDController.hpp>

using namespace ras_group8_motor_controller;
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(10.0);
  
  PIDController pid_controller =
    PIDController::load(node_handle);
  
  MotorController<PIDController> motor_controller =
    MotorController<PIDController>::load(node_handle, pid_controller);
  
  while (node_handle.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
