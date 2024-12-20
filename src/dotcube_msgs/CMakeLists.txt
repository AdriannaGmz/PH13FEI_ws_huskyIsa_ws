cmake_minimum_required(VERSION 2.8.3)
project(dotcube_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation    # for python msgs and services
  genmsg                #  for cpp msgs and srv
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PandarPacket.msg
  PandarScan.msg
  VelodynePacket.msg
  VelodynePositionPacket.msg
  VelodyneScan.msg
  DurationStamped.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
)

###########
## Build ##
###########
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

### Declare a C++ executable
add_executable(TestPublDotcube_node src/testPubl.cpp)
add_executable(TestSubscDotcube_node src/testSubsc.cpp)

### Specify libraries to link a library or executable target against
target_link_libraries(TestPublDotcube_node ${catkin_LIBRARIES})
target_link_libraries(TestSubscDotcube_node ${catkin_LIBRARIES})

## Adding the messages/services libraries 
add_dependencies(TestPublDotcube_node dotcube_msgs_gencpp)
add_dependencies(TestSubscDotcube_node dotcube_msgs_gencpp)


