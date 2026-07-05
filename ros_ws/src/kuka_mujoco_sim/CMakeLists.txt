cmake_minimum_required(VERSION 3.0.2)
project(kuka_mujoco_sim)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs soft_robot_msgs)
catkin_python_setup()
catkin_package()

# Task 8 adds scripts/run_scenarios.py now that the file exists.
# e2e_compliance_smoke.py is the layer-3 automated closed-loop check
# (force -> controller -> motion); installed so `rosrun` can find it.
catkin_install_python(PROGRAMS scripts/sim_node scripts/run_scenarios.py
  scripts/viewer_node
  test/e2e_compliance_smoke.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
endif()
