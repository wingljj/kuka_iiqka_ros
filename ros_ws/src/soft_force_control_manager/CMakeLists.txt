cmake_minimum_required(VERSION 3.0.2)
project(soft_force_control_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_srvs
  actionlib
  diagnostic_msgs
  geometry_msgs
  controller_manager_msgs
  soft_robot_msgs
  soft_force_control_core
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES soft_force_control_manager_core
  CATKIN_DEPENDS roscpp roslib std_srvs actionlib diagnostic_msgs geometry_msgs
                 controller_manager_msgs soft_robot_msgs
                 soft_force_control_core
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Pure-logic manager core: no roscpp runtime dependency.
add_library(soft_force_control_manager_core
  src/system_state_core.cpp
  src/calibration_sequencer.cpp
  src/payload_yaml.cpp
  src/manager_runtime.cpp
)
target_link_libraries(soft_force_control_manager_core ${catkin_LIBRARIES})

add_executable(soft_robot_manager src/manager_node.cpp)
add_dependencies(soft_robot_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(soft_robot_manager soft_force_control_manager_core
                      ${catkin_LIBRARIES})

install(TARGETS soft_robot_manager
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # Package-prefixed gtest target names (global CMake namespace under
  # catkin_make; Plan 4 follow-up 1).
  catkin_add_gtest(test_manager_system_state test/test_system_state_core.cpp)
  target_link_libraries(test_manager_system_state
                        soft_force_control_manager_core
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_manager_calibration_sequencer
                   test/test_calibration_sequencer.cpp)
  target_link_libraries(test_manager_calibration_sequencer
                        soft_force_control_manager_core
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_manager_payload_yaml test/test_payload_yaml.cpp)
  target_link_libraries(test_manager_payload_yaml
                        soft_force_control_manager_core
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_manager_runtime test/test_manager_runtime.cpp)
  target_link_libraries(test_manager_runtime
                        soft_force_control_manager_core pthread
                        ${GTEST_MAIN_LIBRARIES})
endif()
