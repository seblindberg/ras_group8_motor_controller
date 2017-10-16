#include <ros/ros.h>
#include <ras_group8_motor_controller/MotorController.hpp>
#include <ras_group8_motor_controller/StaticController.hpp>
#include <std_msgs/Int32.h>
#include <rosbag/bag.h>

using namespace ras_group8_motor_controller;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "characterizer");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(10.0);
  
  rosbag::Bag bag;
  bag.open("motor_ramp.bag", rosbag::bagmode::Write);
  
  StaticController controller;
  
  MotorController<StaticController> motor_controller =
    MotorController<StaticController>::load(node_handle, controller);
    
  ros::Time time_prev = ros::Time::now();
  ros::Time now;
  double dt;
  double ramp = 0;
  int ramp_dir = 1;
  
  std_msgs::Int32 velocity;
  std_msgs::Int32 target;
  
  while (node_handle.ok()) {
    ros::spinOnce();
    
    now = ros::Time::now();
    
    velocity.data = motor_controller.velocity();
    target.data   = ramp;
    
    bag.write("target", now, target);
    bag.write("velocity", now, velocity);
    
    dt = (now - time_prev).toSec();
    time_prev = now;
    
    ramp = target.data + ramp_dir * dt * 1.0;
    
    if (ramp >= 250) {
      ramp = 250;
      ramp_dir = -1;
    } else if (ramp <= -250) {
      ramp = 250;
      ramp_dir = 1;
    }
    
    motor_controller.setTargetVelocity(ramp);
    
    loop_rate.sleep();
  }
  
  bag.close();

  return 0;
}
