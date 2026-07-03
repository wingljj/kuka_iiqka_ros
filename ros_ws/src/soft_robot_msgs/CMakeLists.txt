cmake_minimum_required(VERSION 3.0.2)
project(soft_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  CartesianState.msg
  CartesianCorrection.msg
  CartesianCorrectionStamped.msg
  RsiState.msg
  ModeCommand.msg
  ModeState.msg
  SriStatus.msg
  EkiState.msg
  ManagerState.msg
)

add_service_files(
  FILES
  GetTool.srv
  SetFilter.srv
  SetEkiMode.srv
  SetToolBase.srv
  StartServo.srv
)

add_action_files(
  FILES
  MoveToOrientation.action
  CalibratePayload.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
)

if(CATKIN_ENABLE_TESTING)
  add_compile_options(-std=c++14 -Wall -Wextra)
  include_directories(${CATKIN_DEVEL_PREFIX}/include ${catkin_INCLUDE_DIRS})
  catkin_add_gtest(test_msgs test/test_msgs.cpp)
  add_dependencies(test_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS})
  target_link_libraries(test_msgs ${GTEST_MAIN_LIBRARIES})
endif()
