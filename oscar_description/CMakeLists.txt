cmake_minimum_required(VERSION 2.8.3)
project(oscar_description)

find_package(catkin REQUIRED COMPONENTS
  urdf roslaunch
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)

install(DIRECTORY launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


