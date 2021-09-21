cmake_minimum_required(VERSION 2.8.3)
project(crazyflie)

set(CMAKE_CXX_FLAGS "-fopenmp")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospack
  genmsg
  sensor_msgs
  message_generation
  eigen_conversions
)


add_subdirectory(lib/collision_vo)

add_message_files(
  FILES
  array3_1.msg
)

add_service_files(
  FILES
  cvo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  #include
  lib/collision_vo/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(cvo_server
  src/cvo_server.cpp)
target_link_libraries(cvo_server
  collision_vo
  ${catkin_LIBRARIES})
# add_dependencies(cvo_server crazyflie_gencpp)
