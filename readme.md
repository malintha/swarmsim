## Setting up the packages

Clone the following packages to source directory of the catkin workspace.

https://github.com/Malintha/drone_demo/tree/multi_mavros

https://github.com/Malintha/sitl_gazebo/tree/xacro_models

https://github.com/Malintha/swarmsim

https://github.com/osrf/uav_testing/tree/typhoon_demo

https://github.com/catkin/catkin_simple

Follow the Installation instructions of https://github.com/Malintha/mav_trajectory_generation. (Make sure you merge the devel and install spaces.) This will clone the following packages.

1. eigen_catkin
2. eigen_checks
3. glog_catkin
4. mav_comm
5. mav_trajectory_generation
6. mav_trajectory_generation_ros
7. mav_trajectory_generation_example
8. mav_visualization
9. nlopt
10. matlab


Use the follwing commands to build the packages

```bash
catkin build mav_trajectory_generation_ros
```
```bash
catkin build
```