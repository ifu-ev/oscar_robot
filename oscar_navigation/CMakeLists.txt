cmake_minimum_required(VERSION 2.8.3)
project(oscar_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

