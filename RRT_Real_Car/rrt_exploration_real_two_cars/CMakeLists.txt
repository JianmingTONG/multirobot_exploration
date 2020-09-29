cmake_minimum_required(VERSION 2.8.3)
project(rrt_exploration_real_two_cars)

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
  actionlib
  actionlib_msgs
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

add_executable(global_rrt_detector_real_two_cars src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(global_rrt_detector_real_two_cars ${catkin_LIBRARIES})

add_executable(local_rrt_detector_real_two_cars src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(local_rrt_detector_real_two_cars ${catkin_LIBRARIES})

add_executable(rrt_node_real_two_cars src/rrt_node.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(rrt_node_real_two_cars ${catkin_LIBRARIES})
