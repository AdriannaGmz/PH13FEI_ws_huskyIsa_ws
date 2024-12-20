cmake_minimum_required(VERSION 2.8.3)
project(flir_adk_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rospy
  nodelet
  cv_bridge
  image_transport
  sensor_msgs
  camera_info_manager
  roslint
  message_generation
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  FlatFieldCorrection.srv 
  Info.srv
  Reboot.srv
  Sync.srv
  )

generate_messages(
   DEPENDENCIES
   std_msgs
   #actionlib_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager
  image_transport
  cv_bridge
  nodelet
  sensor_msgs
  roscpp
  rospy
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/flir_boson_usb_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(BosonCamera
  src/nodelets/BosonCamera.cpp
)

add_dependencies(BosonCamera
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(BosonCamera
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

set(ROSLINT_CPP_OPTS "--filter=-runtime/threadsafe_fn,-build/namespaces,-build/include_what_you_use")
roslint_cpp()

install(TARGETS ${PROJECT_NAME}_node BosonCamera
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
