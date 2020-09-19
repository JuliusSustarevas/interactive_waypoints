cmake_minimum_required(VERSION 2.8.3)

project(interactive_waypoints)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  rospy
  std_msgs
)

#This will set it up in python path
catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs rospy std_msgs
)
