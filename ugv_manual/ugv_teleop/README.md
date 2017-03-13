## Install dependency 
wstool update -t src

## General Description
1. joy is the joystick ROS driver
2. teleop_twist_joy is the bridge between joy (sensor_msgs/Joy) and cmd_vel (geometry_msgs/Twist). Namly, joystick command and vehicle speed.
3.  UGV_telep contains the controller of the PTZ camera.

## Launch
roslaunch ugv_teleop manual_control.launch
