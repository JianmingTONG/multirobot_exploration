cmake_minimum_required(VERSION 2.8.3)
project(apf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  actionlib_msgs
  actionlib
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package( 
CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(apf_node src/apf_node.cpp)
target_link_libraries(apf_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
