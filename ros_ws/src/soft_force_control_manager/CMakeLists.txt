cmake_minimum_required(VERSION 3.0.2)
project(soft_force_control_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  actionlib
  diagnostic_msgs
  geometry_msgs
  controller_manager_msgs
  soft_robot_msgs
  soft_force_control_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES soft_force_control_manager_core
  CATKIN_DEPENDS roscpp std_srvs actionlib diagnostic_msgs geometry_msgs
                 controller_manager_msgs soft_robot_msgs
                 soft_force_control_core
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS})

# Pure-logic manager core: no roscpp runtime dependency.
add_library(soft_force_control_manager_core
  src/system_state_core.cpp
)
target_link_libraries(soft_force_control_manager_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # Package-prefixed gtest target names (global CMake namespace under
  # catkin_make; Plan 4 follow-up 1).
  catkin_add_gtest(test_manager_system_state test/test_system_state_core.cpp)
  target_link_libraries(test_manager_system_state
                        soft_force_control_manager_core
                        ${GTEST_MAIN_LIBRARIES})
endif()
