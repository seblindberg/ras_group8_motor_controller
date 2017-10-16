#include <ros/ros.h>
#include <ras_group8_motor_controller/Characterizer.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "characterizer");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(10.0);

  ras_group8_motor_controller::Characterizer characterizer(node_handle);

  while (node_handle.ok()) {
    ros::spinOnce();
    
    characterizer.update();
    
    loop_rate.sleep();
  }

  return 0;
}
