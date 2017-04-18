cmake_minimum_required(VERSION 2.8.3)
project(ras_lab1_motors)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs ras_lab1_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp geometry_msgs ras_lab1_msgs
  INCLUDE_DIRS include
  LIBRARIES kobuki_motors
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_custom_target(
  copy_kobuki_motors_lib ALL
  COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/libkobuki_motors.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libkobuki_motors.so
)

add_library(kobuki_motors SHARED)
set_target_properties(kobuki_motors PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(kobuki_motors copy_kobuki_motors_lib)

add_executable(kobuki_motors_node src/kobuki_motors_node.cpp)
add_dependencies(kobuki_motors_node kobuki_motors)
target_link_libraries(kobuki_motors_node ${catkin_LIBRARIES} kobuki_motors)

install(TARGETS kobuki_motors_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
