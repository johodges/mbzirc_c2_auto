cmake_minimum_required(VERSION 2.8.3)
project(ugv_teleop)
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_generation
  roscpp
  sensor_msgs
  tf
)
find_package( OpenCV)
find_package( OpenCV REQUIRED )


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${libcurl_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(main_detection src/main_detection.cpp)
target_link_libraries(main_detection ${OpenCV_LIBS} )
target_link_libraries(main_detection ${libcurl_LIBS} )
target_link_libraries(main_detection curl)

#add_executable(sunba_video src/sunba.cpp)
#target_link_libraries(sunba_video
#  ${catkin_LIBRARIES}
#)


