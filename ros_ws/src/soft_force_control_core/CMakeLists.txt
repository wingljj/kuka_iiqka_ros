cmake_minimum_required(VERSION 3.0.2)
project(soft_force_control_core)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

add_compile_options(-std=c++14 -Wall -Wextra)
include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/mode_manager_core.cpp
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mode_manager test/test_mode_manager.cpp)
  target_link_libraries(test_mode_manager ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})
endif()
