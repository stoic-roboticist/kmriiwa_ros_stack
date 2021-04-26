cmake_minimum_required(VERSION 3.0.2)
project(kmriiwa_gazebo)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  gazebo_ros_control
  roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
   DEPENDS
     roscpp
     gazebo_ros
     gazebo_plugins
     hector_gazebo
     gazebo_ros_control    
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
include_directories(
  ${Boost_INCLUDE_DIR}  
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/kmriiwa_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)