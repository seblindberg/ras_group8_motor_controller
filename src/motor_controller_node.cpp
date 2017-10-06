#include <ros/ros.h>
#include <ras_group8_motor_controller/MotorController.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(10.0);
  
  ras_group8_motor_controller::MotorController motor_controller(node_handle);
  
  while (node_handle.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
