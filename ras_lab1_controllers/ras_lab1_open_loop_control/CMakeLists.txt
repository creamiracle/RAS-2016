cmake_minimum_required(VERSION 2.8.3)
project(ras_lab1_open_loop_control)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Declare that this catkin package's runtime dependencies
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(open_loop src/open_loop_controller.cpp)
target_link_libraries(open_loop ${catkin_LIBRARIES})
add_dependencies(open_loop ${catkin_EXPORTED_TARGETS})