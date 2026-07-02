cmake_minimum_required(VERSION 3.0.2)
project(kuka_rsi_hw_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  realtime_tools
  soft_robot_msgs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuka_rsi_protocol
  CATKIN_DEPENDS roscpp hardware_interface controller_manager realtime_tools
                 soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIRS})

# Protocol / session library: no roscpp runtime dependency, fully unit-testable.
add_library(kuka_rsi_protocol
  src/rsi_frame.cpp
  src/rsi_session_monitor.cpp
  src/command_limiter.cpp
  src/udp_transport.cpp
)
target_link_libraries(kuka_rsi_protocol ${TINYXML2_LIBRARIES})

add_library(kuka_rsi_hw_interface_mock
  src/rsi_mock_core.cpp
  src/rsi_mock_server.cpp
)
target_link_libraries(kuka_rsi_hw_interface_mock kuka_rsi_protocol pthread)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_rsi_frame_parse test/test_rsi_frame_parse.cpp)
  target_link_libraries(test_rsi_frame_parse kuka_rsi_protocol
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_rsi_frame_serialize test/test_rsi_frame_serialize.cpp)
  target_link_libraries(test_rsi_frame_serialize kuka_rsi_protocol
                        ${TINYXML2_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_rsi_session_monitor test/test_rsi_session_monitor.cpp)
  target_link_libraries(test_rsi_session_monitor kuka_rsi_protocol
                        ${GTEST_MAIN_LIBRARIES})
  catkin_add_gtest(test_command_limiter test/test_command_limiter.cpp)
  target_link_libraries(test_command_limiter kuka_rsi_protocol
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_udp_transport test/test_udp_transport.cpp)
  target_link_libraries(test_udp_transport kuka_rsi_protocol
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_cartesian_interfaces test/test_cartesian_interfaces.cpp)
  target_link_libraries(test_cartesian_interfaces ${catkin_LIBRARIES}
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_rsi_mock test/test_rsi_mock.cpp)
  target_link_libraries(test_rsi_mock kuka_rsi_hw_interface_mock
                        ${GTEST_MAIN_LIBRARIES})
endif()
