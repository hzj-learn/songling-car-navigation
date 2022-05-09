cmake_minimum_required(VERSION 2.8.3)
project(car_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  autoware_msgs
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS autoware_msgs
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_executable(car_controller src/car_controller.cpp)

add_dependencies(car_controller car_controller_generate_messages_cpp)

target_link_libraries(car_controller ${catkin_LIBRARIES} libcontrolcan.so)


