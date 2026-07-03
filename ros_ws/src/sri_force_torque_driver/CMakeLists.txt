cmake_minimum_required(VERSION 3.0.2)
project(sri_force_torque_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_srvs
  soft_force_control_core
  soft_robot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sri_ft_protocol sri_ft_mock
  CATKIN_DEPENDS roscpp geometry_msgs std_srvs soft_force_control_core
                 soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS})

# Protocol / session library: no roscpp runtime dependency.
add_library(sri_ft_protocol
  src/sri_frame.cpp
  src/tcp_client_transport.cpp
  src/sri_stream_session.cpp
)
target_link_libraries(sri_ft_protocol ${catkin_LIBRARIES})

# after the sri_ft_protocol block
add_library(sri_ft_mock
  src/sri_mock_server.cpp
)
target_link_libraries(sri_ft_mock sri_ft_protocol pthread)

add_executable(sri_mock_server src/sri_mock_server_main.cpp)
target_link_libraries(sri_mock_server sri_ft_mock)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_sri_frame test/test_sri_frame.cpp)
  target_link_libraries(test_sri_frame sri_ft_protocol ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_tcp_client_transport test/test_tcp_client_transport.cpp)
  target_link_libraries(test_tcp_client_transport sri_ft_protocol
                        ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_sri_stream_session test/test_sri_stream_session.cpp)
  target_link_libraries(test_sri_stream_session sri_ft_protocol
                        ${catkin_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_sri_mock_server test/test_sri_mock_server.cpp)
  target_link_libraries(test_sri_mock_server sri_ft_mock
                        ${GTEST_MAIN_LIBRARIES})
endif()
