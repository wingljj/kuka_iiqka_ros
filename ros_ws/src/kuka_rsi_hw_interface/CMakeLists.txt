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
  LIBRARIES kuka_rsi_protocol kuka_rsi_robot_hw
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

add_library(kuka_rsi_robot_hw
  src/kuka_rsi_robot_hw.cpp
)
target_link_libraries(kuka_rsi_robot_hw kuka_rsi_protocol ${catkin_LIBRARIES})

add_library(kuka_rsi_hw_interface_mock
  src/rsi_mock_core.cpp
  src/rsi_mock_server.cpp
)
target_link_libraries(kuka_rsi_hw_interface_mock kuka_rsi_protocol pthread)

add_executable(kuka_rsi_sim_server src/kuka_rsi_sim_server_main.cpp)
target_link_libraries(kuka_rsi_sim_server kuka_rsi_hw_interface_mock)

add_executable(kuka_rsi_hw_interface_node src/kuka_rsi_hw_interface_node.cpp)
add_dependencies(kuka_rsi_hw_interface_node
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(kuka_rsi_hw_interface_node kuka_rsi_robot_hw
                      ${catkin_LIBRARIES})

install(TARGETS kuka_rsi_sim_server kuka_rsi_hw_interface_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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

  catkin_add_gtest(test_kuka_rsi_robot_hw test/test_kuka_rsi_robot_hw.cpp)
  target_link_libraries(test_kuka_rsi_robot_hw kuka_rsi_robot_hw
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_rsi_integration test/test_rsi_integration.cpp)
  target_link_libraries(test_rsi_integration kuka_rsi_robot_hw
                        kuka_rsi_hw_interface_mock ${GTEST_MAIN_LIBRARIES})
endif()
