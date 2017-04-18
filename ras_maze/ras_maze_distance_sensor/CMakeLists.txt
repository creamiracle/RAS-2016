cmake_minimum_required(VERSION 2.8.3)
project(ras_maze_distance_sensor)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf kdl_conversions ras_lab1_msgs)
find_package(Boost REQUIRED COMPONENTS random)
find_package(orocos_kdl REQUIRED)

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

catkin_package(
  DEPENDS boost orocos_kdl
  CATKIN_DEPENDS roscpp std_msgs tf kdl_conversions ras_lab1_msgs
  INCLUDE_DIRS include
  LIBRARIES distance_sensor_maze
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

add_custom_target(
  copy_distance_sensor_lib_maze ALL
  COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/libdistance_sensor_maze.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libdistance_sensor_maze.so
)

add_library(distance_sensor_maze SHARED)
set_target_properties(distance_sensor_maze PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(distance_sensor_maze copy_distance_sensor_lib_maze)

add_executable(generate_distance_node_maze include/maze_distance_sensor/distance_sensor.h src/generate_distance_node.cpp)
add_dependencies(generate_distance_node_maze distance_sensor_maze)
target_link_libraries(generate_distance_node_maze ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} distance_sensor_maze)

install(TARGETS generate_distance_node_maze
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
