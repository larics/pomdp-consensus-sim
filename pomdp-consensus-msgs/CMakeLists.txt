cmake_minimum_required(VERSION 2.8.3)
project(pomdp_consensus_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

add_message_files(
  FILES
  BeliefStamped.msg
)

add_service_files(
  FILES
  GetNewAction.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
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
