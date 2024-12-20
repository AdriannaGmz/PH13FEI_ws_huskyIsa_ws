cmake_minimum_required(VERSION 2.8.3)
project(test_compare_timestamps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs 
  nav_msgs 
  tf2_msgs
  diagnostic_msgs
  message_generation    # for python msgs and services
  genmsg                #  for cpp msgs and srv
)

# Add C++11 norm and features (like unique_ptr)
add_definitions( -std=c++0x )

generate_messages(    #for srvs and msgs
  DEPENDENCIES
  std_msgs
  sensor_msgs 
  nav_msgs 
  tf2_msgs
  diagnostic_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs tf2_msgs diagnostic_msgs message_runtime 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
  src/test_compare_timestamps.py
  src/*
  src/helloMsgPubSub.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test_compare_timestamps
)

