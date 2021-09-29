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


find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)

add_subdirectory(lib/collision_vo)
add_subdirectory(lib/common_cpp)
add_subdirectory(lib/geometry)


# add_message_files(
#   FILES
#   array3_1.msg
# )

add_service_files(
  FILES
  cvo.srv
  add_subscriber.srv
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
  include
  lib/collision_vo/include
  lib/geometry/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(cvo_server
  src/cvo_server.cpp
  src/quad_wp_manager.cpp)
target_link_libraries(cvo_server
  collision_vo
  common_cpp
  geometry
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES})
# add_dependencies(cvo_server crazyflie_gencpp)
