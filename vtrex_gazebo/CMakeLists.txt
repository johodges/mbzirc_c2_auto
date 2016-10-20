cmake_minimum_required(VERSION 2.8.3)
project(vtrex_gazebo)

find_package(catkin REQUIRED COMPONENTS
roslaunch
  gazebo_ros
 cv_bridge
 sensor_msgs
 roscpp
 image_transport
 std_msgs
 message_generation
)
catkin_package()


install(DIRECTORY Media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}

)


