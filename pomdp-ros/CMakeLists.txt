cmake_minimum_required(VERSION 2.8.3)
project(pomdp_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
 CATKIN_DEPENDS rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
