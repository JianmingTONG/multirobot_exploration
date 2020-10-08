# Overview
This repo consists of:
- gazebo_ubuntu18.04 contains support files to connect frame "robot1/odom" to frame "robot/base_footprint" in ubuntu18.04

## gazebo_ubuntu18.04
All files in gazebo_ubuntu18.04 are catkin packages. Please create a catkin workspace. Put files in gazebo_ubuntu18.04 under the src directory of catkin workspace. And compile the folder using "catkin_make".

Put ``` source <catkin_workspace>/devel/setup.bash``` in your ~/.bashrc file. Or ```echo "source <catkin_workspace>/devel/setup.bash" >> ~/.bashrc``` 

Then run 

```
source ~/.bashrc
```


Have Fun :)
