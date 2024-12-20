cmake_minimum_required(VERSION 2.8.3)
project(chrony_test)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg                #  for cpp msgs and srv
)

# Add C++11 norm and features (like unique_ptr)
add_definitions( -std=c++0x )

###################################
## catkin specific configuration ##
###################################
catkin_package(
)


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(publCurrTime_node src/publCurrTime.cpp)
add_executable(publCurrTime_speed_node src/publCurrTime_speed.cpp)
add_executable(publCurrTime_aislap58_node src/publCurrTime_aislap58.cpp)
add_executable(subscCurrTime_node src/subscCurrTime.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(publCurrTime_node ${catkin_LIBRARIES})
target_link_libraries(publCurrTime_speed_node ${catkin_LIBRARIES})
target_link_libraries(publCurrTime_aislap58_node ${catkin_LIBRARIES})
target_link_libraries(subscCurrTime_node ${catkin_LIBRARIES})

