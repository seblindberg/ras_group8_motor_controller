#include <ros/ros.h>

class MotorControllerNode
{
public:
  MotorControllerNode()
  {
    n_ = ros::NodeHandle("~");
    ROS_INFO("Motor Controller is loaded");
  }

  void update()
  {
    
  }

  bool ok() {
    return n_.ok();
  }

private:

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");

  double control_frequency = 10.0;

  MotorControllerNode controller = MotorControllerNode();
  ros::Rate loop_rate(control_frequency);

  while (controller.ok())
  {
      ros::spinOnce();
      controller.update();

      loop_rate.sleep();
  }

  return 0;
}
