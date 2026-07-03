cmake_minimum_required(VERSION 3.0.2)
project(soft_robot_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  realtime_tools
  actionlib
  pluginlib
  geometry_msgs
  soft_force_control_core
  kuka_rsi_hw_interface
  soft_robot_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp controller_interface hardware_interface
                 realtime_tools actionlib pluginlib geometry_msgs
                 soft_force_control_core kuka_rsi_hw_interface
                 soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mode_bridge test/test_mode_bridge.cpp)
  add_dependencies(test_mode_bridge ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_mode_bridge ${GTEST_MAIN_LIBRARIES})
endif()
