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
  src/force_torque_filter.cpp
  src/tool_gravity_compensator.cpp
  src/compliance_law.cpp
  src/safety_limiter.cpp
  src/adaptive_deadband.cpp
  src/auto_retare.cpp
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mode_manager test/test_mode_manager.cpp)
  target_link_libraries(test_mode_manager ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_filter test/test_filter.cpp)
  target_link_libraries(test_filter ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_rotation test/test_rotation.cpp)
  target_link_libraries(test_rotation ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_gravity_compensator test/test_gravity_compensator.cpp)
  target_link_libraries(test_gravity_compensator ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_compliance_law test/test_compliance_law.cpp)
  target_link_libraries(test_compliance_law ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_safety_limiter test/test_safety_limiter.cpp)
  target_link_libraries(test_safety_limiter ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_adaptive_deadband test/test_adaptive_deadband.cpp)
  target_link_libraries(test_adaptive_deadband ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})

  catkin_add_gtest(test_auto_retare test/test_auto_retare.cpp)
  target_link_libraries(test_auto_retare ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})
endif()
