## Setting up the packages

Clone the following packages to source directory of the catkin workspace.
```bash
git clone https://github.com/malintha/drone_demo.git -b xacro_models

git clone https://github.com/malintha/sitl_gazebo.git -b xacro_merge --recursive

git clone https://github.com/malintha/uav_testing.git -b master
```

Run following commands from the root of the workspace. Please change the ros distribution name as suitable.

```bash
source /opt/ros/melodic/setup.sh && rosdep update && rosdep install --from-path src -iy

source /opt/ros/melodic/setup.sh && catkin config --install

catkin build

git clone https://github.com/Malintha/swarmsim -b master

git clone https://github.com/catkin/catkin_simple
```

Follow the Installation instructions of https://github.com/Malintha/mav_trajectory_generation. (Make sure you merge the devel space.) This will clone the following packages.

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
catkin build swarmsim_example
```