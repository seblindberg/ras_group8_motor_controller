#include <ros/ros.h>
#include <ras_group8_motor_controller/MotorController.hpp>
#include <ras_group8_motor_controller/PIDController.hpp>

using namespace ras_group8_motor_controller;
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle node_handle("~");
  
  /* Load the loop update rate.
     Default: 10 Hz  */
  double update_rate = node_handle.param("update_rate", 10.0);
  ros::Rate loop_rate(update_rate);
  
  /* Setup the PID controller with values stored in the
     parameters server. */
  PIDController pid_controller =
    PIDController::load(node_handle);
  
  /* Setup the MotorController, using the PID controller
     with values stored in the parameter server. */
  MotorController<PIDController> motor_controller =
    MotorController<PIDController>::load(node_handle, pid_controller);
  
  while (node_handle.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  motor_controller.shutdown();
  
  return 0;
}
