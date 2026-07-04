cmake_minimum_required(VERSION 3.0.2)
project(kuka_mujoco_sim)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs soft_robot_msgs)
catkin_python_setup()
catkin_package()

# Task 8 adds scripts/run_scenarios.py now that the file exists.
catkin_install_python(PROGRAMS scripts/sim_node scripts/run_scenarios.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
endif()
