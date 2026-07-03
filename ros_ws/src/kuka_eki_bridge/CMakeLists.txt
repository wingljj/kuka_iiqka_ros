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
  LIBRARIES kuka_eki_protocol
  CATKIN_DEPENDS roscpp std_srvs diagnostic_msgs soft_robot_msgs
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${catkin_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIRS})

# Protocol / session library: no roscpp runtime dependency.
add_library(kuka_eki_protocol
  src/eki_frame.cpp
  src/eki_stream_splitter.cpp
)
target_link_libraries(kuka_eki_protocol ${TINYXML2_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_eki_frame test/test_eki_frame.cpp)
  target_link_libraries(test_eki_frame kuka_eki_protocol
                        ${GTEST_MAIN_LIBRARIES})
endif()
