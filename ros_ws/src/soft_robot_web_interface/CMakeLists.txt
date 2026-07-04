cmake_minimum_required(VERSION 3.0.2)
project(soft_robot_web_interface)

find_package(catkin REQUIRED)
catkin_package()

# Static frontend + launch + serving shim. No compiled targets; the JS
# unit tests run through `node --test test/unit/` (not part of gtest).
install(DIRECTORY www launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/serve_www.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
