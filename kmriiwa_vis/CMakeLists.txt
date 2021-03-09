cmake_minimum_required(VERSION 3.0.2)
project(kmriiwa_vis)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
