cmake_minimum_required(VERSION 2.8.3)
project(base_vel_mux)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp   dynamic_reconfigure geometry_msgs)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/reload.cfg)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp   dynamic_reconfigure geometry_msgs
   DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})



add_library(base_vel_mux
  src/base_vel_mux.cpp
)
add_dependencies(base_vel_mux ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_vel_mux ${yaml-cpp_LIBRARIES} cmd_vel_subscribers
  ${catkin_LIBRARIES}
)

add_executable(base_vel_mux_node
  src/base_vel_mux_node.cpp
)
add_dependencies(base_vel_mux_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_vel_mux_node
  ${catkin_LIBRARIES} base_vel_mux
)

add_library(cmd_vel_subscribers
  src/cmd_vel_subscribers.cpp
)
add_dependencies(cmd_vel_subscribers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_subscribers
  ${catkin_LIBRARIES}
)
