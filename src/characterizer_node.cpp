#include <ros/ros.h>
#include <signal.h>
#include <ras_group8_motor_controller/MotorController.hpp>
#include <ras_group8_motor_controller/StaticController.hpp>

#define MOTOR_MIN -50
#define MOTOR_MAX 50

using namespace ras_group8_motor_controller;

static MotorController<StaticController> *controller;

void quit(int sig)
{
  controller->shutdown();
  ROS_INFO("Killing motor driver");
  
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "characterizer");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(10.0);
  
  StaticController static_controller;
  
  MotorController<StaticController> motor_controller =
    MotorController<StaticController>::load(node_handle, static_controller);
  
  ros::Time time_prev = ros::Time::now();
  ros::Time now;
  double dt;
  
  double ramp = 0;
  int ramp_dir = 1;
  
  /* Trap signal exit. */
  controller = &motor_controller;
  signal(SIGINT, quit);
  
  while (node_handle.ok()) {
    ros::spinOnce();
    
    now = ros::Time::now();
    
    dt = (now - time_prev).toSec();
    time_prev = now;
    
    ramp += ramp_dir * dt * 5.0;
    
    if (ramp >= MOTOR_MAX) {
      ramp = MOTOR_MAX;
      ramp_dir = -1;
    } else if (ramp <= MOTOR_MIN) {
      ramp = MOTOR_MIN;
      ramp_dir = 1;
    }
    
    motor_controller.setTargetVelocity(ramp);
    ROS_INFO("Set value = %f", ramp);
    
    loop_rate.sleep();
  }

  return 0;
}
