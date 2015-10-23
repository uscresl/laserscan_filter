cmake_minimum_required(VERSION 2.8.3)
project(laserscan_filter_nodelet)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_filters
  tf
)

include_directories(${catkin_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laserscan_filter_nodelet
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nodelet message_filters tf
#  DEPENDS system_lib
)

## Create the nodelet library
add_library(laserscan_filter_nodelet src/laserscan_filter_nodelet.cpp)
target_link_libraries(laserscan_filter_nodelet ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(laserscan_filter_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS laserscan_filter_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES laserscan_filter_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

