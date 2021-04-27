# KMRIIWA_ROS_STACK
This metapackage enables to use and control the KMRIIWA robot using ROS. It provides packages for robot's navigation, manipulation, visualisation and simulation. A brief description of thsee packages and a usage guide follows. 

**Note that this package depends on the [kmriiwa_ros_java driver](https://github.com/stoic-roboticist/kmriiwa_ros_java) to be running on the robot for its control.**

**This pakcage was tested on Ubuntu 20.04 with ROS noetic distro**

## Description

### kmriiwa_bringup
This package bringups the robot description and its tf transformations to the ros network. It also allows to bringup the full planning stack of the robot.

### kmriiwa_description
This package contains the robot's URDFs, meshes and Gazebo descriptions.

### kmriiwa_moveit
This package allows moveit to control the LBR-IIWA14 arm. Currently, this package only utilises a FollowJointTrajectory action server running on the robot to execute manipulation trajectories. No low level ros_control is available. This is limitation is explained in [kmriiwa_ros_java driver](https://github.com/stoic-roboticist/kmriiwa_ros_java).

### kmriiwa_navigation
This package allows to control the KMP200 base using the navigation stack. It utilises move_base, amcl, teb_local_planner and ros_local_planner packages to control the robot. In addition, gmapping package is used to create SLAM maps.

### kmriiwa_msgs
This package contains custom ros messages specific to the KMRIIWA robot that allows to convey its status.

### kmriiwa_gazebo
This package allows to simulate the robot in Gazebo. It exposes the same interfaces as the robot driver and allows to control both the base and the arm.

### kmriiwa_vis
This package contain different rviz configs to visualise the robot in different contexts.

## Usage

**To run this package, [kmriiwa_ros_java driver](https://github.com/stoic-roboticist/kmriiwa_ros_java) needs to be installed on the robot controller. Refer to that package README.md for setup and usage instructions.**
### robot bringup
Once the [kmriiwa_ros_java driver](https://github.com/stoic-roboticist/kmriiwa_ros_java) is up and running on the robot controller. Launch the robot bringup package to upload the robot description and publish its tf information:
```
roslaunch kmriiwa_bringup kmriiwa_bringup.launch
```
### navigation and manipulation planning stack
After that the robot is ready to be used by moveit and the navigation stack. To launch both stacks at the same time use:
```
roslaunch kmriiwa_bringup planning_stack_bringup.launch rviz:=<true|false> no_static_map:=<true|false> map_file:=<path/to/map/file>
```
If the *rviz* argument is set to true, rviz will launch and can be used to control the robot. If the robot is to be used in a known environment, the argument *no_static_map* need to be set to false and the argument *map_file* need to be specified. Otherwise, it is not used.

### moveit
To use it moveit, run the command:
```
roslaunch kmriiwa_moveit move_group.launch
```
To bringup rviz with moveit settings use:
```
roslaunch kmriiwa_vis moveit_view.launch
```

### navigation stack
To use the navigaion stack in an unknown map use:
```
roslaunch kmriiwa_navigation mapless_navigation.launch rviz:=<true|false>
```
If the *rviz* argument is set to true, rviz will launch and can be used to navigate the robot.

To create a map for navigation start gmapping first:
```
roslaunch kmriiwa_navigation gmapping.launch
```
Then jog the real robot in your map using the SmartPad while trying to cover most of the space in slow controlled motions. When done with robot jogging in the map, in a new terminal `cd` to your prefered directory and run
```
rosrun map_server map_server -f <map_name>
```
This should save the map to your disk. Now you can stop the gmapping node.

To use the navigaion stack in an known map use:
```
roslaunch kmriiwa_navigation map_navigation.launch rviz:=<true|false> map_file:=<path/to/map/file>
```
If the *rviz* argument is set to true, rviz will launch and can be used to navigate the robot.

Finally, each of these launch files accept other arguments not mentioned here, refere to these files for more information. Moreover, the various navigation configs that include global costmap, local costmap and local planner can be reconfigured if needed.

### gazebo simulation
To launch the robot in gazebo simulation in empty world use:
```
roslaunch kmriiwa_gazebo kmriiwa_empty_world.launch
```
A more interesting world to launch is test_zone. To use this world:
```
roslaunch kmriiwa_gazebo kmriiwa_test_zone.launch
```
After launching any of the previous worlds, the combined planning stack (planning_stack_bringup.launch) can be launched to start controlling the simulated robot. Alternatively, to launch moveit or the navigation stack individually, use any of the previously described commands.
