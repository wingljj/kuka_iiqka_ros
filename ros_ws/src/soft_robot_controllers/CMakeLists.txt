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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp controller_interface hardware_interface
                 realtime_tools actionlib pluginlib geometry_msgs
                 soft_force_control_core kuka_rsi_hw_interface
                 soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/controller_mode_gate.cpp
  src/force_compliance_core.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mode_bridge test/test_mode_bridge.cpp)
  add_dependencies(test_mode_bridge ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_mode_bridge ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_controller_mode_gate test/test_controller_mode_gate.cpp)
  target_link_libraries(test_controller_mode_gate ${PROJECT_NAME}
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_force_compliance_core test/test_force_compliance_core.cpp)
  target_link_libraries(test_force_compliance_core ${PROJECT_NAME}
                        ${GTEST_MAIN_LIBRARIES})
endif()
