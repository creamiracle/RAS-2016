cmake_minimum_required(VERSION 2.8.3)
project(ras_lab1_world)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs kdl_conversions)
find_package(Boost REQUIRED COMPONENTS random)
find_package(orocos_kdl REQUIRED)

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS roscpp std_msgs kdl_conversions
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(world_node src/world_main.cpp)
target_link_libraries(world_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS world_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
