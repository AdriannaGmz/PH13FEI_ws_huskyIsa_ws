cmake_minimum_required(VERSION 3.0.2)
project(husky_isa4_init)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  tf
)
catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
# add_executable(tf_broadcaster src/tf_broadcaster.cpp)
# add_executable(tf_broadcaster_lidar src/tf_broadcaster_lidar.cpp)
# add_executable(tf_listener src/tf_listener.cpp)
# add_executable(tf2_broadcaster_lidar src/tf2_broadcaster_lidar.cpp)
add_executable(tf2_static_lidar src/tf2_static_lidar.cpp)
add_executable(tf2_kams_dot src/tf2_kams_dot.cpp)
add_executable(tf2_kams_irkam src/tf2_kams_irkam.cpp)
add_executable(tf2_kams_cam0l src/tf2_kams_cam0L.cpp)
add_executable(tf2_kams_cam1r src/tf2_kams_cam1R.cpp)
add_executable(tf2_kams_camth src/tf2_kams_camTh.cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
# target_link_libraries(tf_broadcaster_lidar ${catkin_LIBRARIES})
# target_link_libraries(tf_listener ${catkin_LIBRARIES})
# target_link_libraries(tf2_broadcaster_lidar ${catkin_LIBRARIES})
target_link_libraries(tf2_static_lidar ${catkin_LIBRARIES})
target_link_libraries(tf2_kams_dot ${catkin_LIBRARIES})
target_link_libraries(tf2_kams_irkam ${catkin_LIBRARIES})
target_link_libraries(tf2_kams_cam0l ${catkin_LIBRARIES})
target_link_libraries(tf2_kams_cam1r ${catkin_LIBRARIES})
target_link_libraries(tf2_kams_camth ${catkin_LIBRARIES})



