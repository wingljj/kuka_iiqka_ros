cmake_minimum_required(VERSION 3.0.2)
project(soft_robot_bringup)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
install(PROGRAMS scripts/load_payload_then_exec.sh scripts/run_mock.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
