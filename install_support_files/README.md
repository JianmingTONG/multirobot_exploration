# Overview
This repo consists of:
- gazebo_ubuntu18.04 contains support files to connect frame "robot1/odom" to frame "robot/base_footprint" in ubuntu18.04

## gazebo_ubuntu18.04 installation
1. create a directory. (anyplace; named catkin_ws)
2. put files of gazebo_ubuntu18.04 in directory catkin_ws
3. cd catkin_ws
4. run ``` catkin_make ```. If catkin complains about miss package, let's say the name of missed package is <missed_package_name>. just install this package using ``` sudo apt install ros-melodic-<missed_package_name>```.
5. run ```echo "source <path2catkin_ws>/catkin_ws/devel/setup.bash" >> ~/.bashrc```
6. run ``` source ~/.bashrc ```


Have Fun :)
