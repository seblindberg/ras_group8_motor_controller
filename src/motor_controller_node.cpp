#include <ros/ros.h>
#include <ras_group8_motor_controller/MotorController.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(10.0);
  
  ras_group8_motor_controller::MotorController motorController(nodeHandle);
  
  //ros::spin();
  for (;;) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
