cmake_minimum_required(VERSION 2.8.3)
project(crazyflie)

set(CMAKE_CXX_FLAGS "-fopenmp")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)
