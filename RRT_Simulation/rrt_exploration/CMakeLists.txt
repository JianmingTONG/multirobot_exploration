cmake_minimum_required(VERSION 2.8.3)
project(rrt_exploration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  actionlib_msgs
  actionlib
)


add_message_files(
  FILES
  PointArray.msg
  PointStampedArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)


catkin_package( 
CATKIN_DEPENDS message_runtime actionlib_msgs
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(local_rrt_detector ${catkin_LIBRARIES})

add_executable(rrt_node src/rrt_node.cpp src/mtrand.cpp src/functions.cpp)
target_link_libraries(rrt_node ${catkin_LIBRARIES})
