cmake_minimum_required(VERSION 2.8.3)
project(receiverSoftware)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_generation    # for python msgs and services
  genmsg                #  for cpp msgs and srv
)

# Add C++11 norm and features (like unique_ptr)
add_definitions( -std=c++0x )


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  audio_with_timestamp.msg
  audio_packet.msg
  doa_result.msg
)

generate_messages(    #for srvs and msgs
  DEPENDENCIES
  std_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs message_runtime 
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(simpleSubsc_node src/simpleSubsc.cpp)
add_executable(audioMarker_node src/pubAudioMarker.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(simpleSubsc_node ${catkin_LIBRARIES})
target_link_libraries(audioMarker_node ${catkin_LIBRARIES})

# Adding the messages/services libraries 
add_dependencies(simpleSubsc_node receiverSoftware_gencpp)
add_dependencies(audioMarker_node receiverSoftware_gencpp)

