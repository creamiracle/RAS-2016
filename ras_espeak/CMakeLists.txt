cmake_minimum_required(VERSION 2.8.3)
project(ras_espeak)

find_package(catkin REQUIRED COMPONENTS)


catkin_package()

install(PROGRAMS scripts/espeak_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
