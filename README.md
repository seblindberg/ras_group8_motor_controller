# Motor Controller

This ROS package implements a motor controller for the RAS 2017 course. The sole task of the controller node is to control the velocity of one motor.

The velocity in m/s is set by publishing a `std_msgs::Float32` to the `velocity_topic` set in the config.