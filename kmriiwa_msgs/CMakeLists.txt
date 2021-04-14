cmake_minimum_required(VERSION 3.0.2)
project(kmriiwa_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
actionlib_msgs
message_generation
cmake_modules
geometry_msgs
std_msgs
sensor_msgs
)

add_message_files(DIRECTORY msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs)