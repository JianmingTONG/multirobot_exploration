# Rapidly-exploration Random Tree (RRT) Deployment on single car in the unknown environment.
## Summary
This repo enables a single car to explore unknown environment autonomously using the RRT algorithm.

## Environment
Car: xtark robot with SLAMTech A1 RPLIDAR. (Supporting codes are stored in the xtark subdirectory.)
PC: 
- ubuntu 18.04
- ROS melodic
- [Original RRT code](https://github.com/hasauino/rrt_exploration)
- Put the xtark_ros_ws into src directory of a catkin package. Using Catkin to build the package and then add source into your ~/.bashrc file.

Note: The debugged RRT code is located at the rrt_exploration_real_car subdirectory.

## Ready to run
1. setup a catkin_workspace and put this repo under the src directory.
2. Use catkin_make to build the directory.
3. setup the devel/setup.bash (add the following code in your ~/.bashrc)
```
export IPAddress=`ifconfig $interface | grep -o 'inet [^ ]*' | cut -d " " -f2`
alias sshrobot2='ssh xtark@192.168.31.140'
alias sshrobot='ssh xtark@192.168.31.201'
export ROS_IP=IPAddress
export ROS_MASTER_URI=IPAddress:11311

```

4. run the following code in different terminal.

```
// New Terminal
ssh xtark@<robot_ip>
roslaunch xtark_nav xtark_mapping_cartographer_robot1_jianming.launch

// New Terminal
roslaunch rrt_exploration_real_two_cars single.launch.
// Using Publish Point in RVIZ to click 5 points (the first points are to specify the range of exploration. the last point is the expected target location of the robot.)
```

For two robots
```
// New Terminal Connect to robot1
ssh xtark@<robot1_ip>
roslaunch xtark_nav xtark_mapping_cartographer_robot1_jianming.launch

// New Terminal Connect to robot2
ssh xtark@<robot2_ip>
roslaunch xtark_nav xtark_mapping_cartographer_robot1_jianming.launch

// New Terminal
roslaunch mapmerge mapmerge_real_two_cars.launch

// New Terminal
roslaunch rrt_exploration_real_two_cars two_robots_real_car.launch

```

Have Fun :)
