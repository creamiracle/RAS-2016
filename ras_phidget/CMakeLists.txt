cmake_minimum_required(VERSION 2.4.6)
project(phidgets)
find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  sensor_msgs
  sound_play
  move_base_msgs
  actionlib
  rviz
  message_generation
  roscpp_serialization
)
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   motor_params.msg
   motor_encoder.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   lidar_control.srv
 )

generate_messages(
 DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs tf sound_play move_base_msgs actionlib rviz message_runtime roscpp_serialization
  DEPENDS
)

add_executable(motor src/motor.cpp)
add_dependencies(motor phidgets_generate_messages_cpp)

target_link_libraries(motor phidget21 ${catkin_LIBRARIES})
