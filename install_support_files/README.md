# Overview
This repo consists of:
- kobuki_desktop-melodic contains support files to connect frame "robot1/odom" to frame "robot/base_footprint" in ubuntu18.04

## install the following dependence on ubuntu 18.04
```
sudo apt install ros-melodic-ecl*
sudo apt install ros-melodic-slam-gmapping


```

## gazebo_ubuntu18.04 installation
1. create a directory. (anyplace; named catkin_ws)
2. put files of kobuki_desktop-melodic in directory catkin_ws
3. cd catkin_ws
4. run ``` catkin_make ```. If catkin complains about miss package, let's say the name of missed package is <missed_package_name>. just install this package using ``` sudo apt install ros-melodic-<missed_package_name>```.
5. run ```echo "source <path2catkin_ws>/catkin_ws/devel/setup.bash" >> ~/.bashrc```
6. run ``` source ~/.bashrc ```


Have Fun :)
