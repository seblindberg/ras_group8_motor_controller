#include <ros/ros.h>
#include <ras_group8_motor_controller/MotorController.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nodeHandle("~");
  
  ras_group8_motor_controller::MotorController motorController(nodeHandle);
    
  ros::spin();
  return 0;
}
