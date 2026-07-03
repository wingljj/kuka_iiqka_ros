cmake_minimum_required(VERSION 3.0.2)
project(kuka_eki_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  diagnostic_msgs
  soft_robot_msgs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuka_eki_protocol kuka_eki_mock
  CATKIN_DEPENDS roscpp std_srvs diagnostic_msgs soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIRS})

# Protocol / session library: no roscpp runtime dependency.
add_library(kuka_eki_protocol
  src/eki_frame.cpp
  src/eki_stream_splitter.cpp
  src/tcp_client_transport.cpp
  src/eki_session_core.cpp
)
target_link_libraries(kuka_eki_protocol ${TINYXML2_LIBRARIES})

add_library(kuka_eki_mock
  src/eki_mock_server.cpp
)
target_link_libraries(kuka_eki_mock kuka_eki_protocol pthread)

add_executable(eki_mock_server src/eki_mock_server_main.cpp)
target_link_libraries(eki_mock_server kuka_eki_mock)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_eki_frame test/test_eki_frame.cpp)
  target_link_libraries(test_eki_frame kuka_eki_protocol
                        ${GTEST_MAIN_LIBRARIES})

  # Named test_eki_tcp_client_transport (not test_tcp_client_transport):
  # catkin_make builds one CMake project, so gtest target names are
  # global and would collide with the sri_force_torque_driver target.
  catkin_add_gtest(test_eki_tcp_client_transport
                   test/test_tcp_client_transport.cpp)
  target_link_libraries(test_eki_tcp_client_transport kuka_eki_protocol
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_eki_session_core test/test_eki_session_core.cpp)
  target_link_libraries(test_eki_session_core kuka_eki_protocol
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_eki_mock_server test/test_eki_mock_server.cpp)
  target_link_libraries(test_eki_mock_server kuka_eki_mock
                        ${GTEST_MAIN_LIBRARIES})
endif()
