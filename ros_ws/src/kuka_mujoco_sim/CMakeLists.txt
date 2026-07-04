cmake_minimum_required(VERSION 3.0.2)
project(kuka_mujoco_sim)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs soft_robot_msgs)
catkin_python_setup()
catkin_package()

# Task 8 will add scripts/run_scenarios.py to this PROGRAMS list; it is
# omitted here because catkin_install_python errors at configure time on a
# missing file, which would break the build before Task 8 lands.
catkin_install_python(PROGRAMS scripts/sim_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
endif()
