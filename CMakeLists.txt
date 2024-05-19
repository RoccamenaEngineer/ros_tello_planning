cmake_minimum_required(VERSION 3.0.2)
project(ros_tello_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  message_filters
  tf
  dynamic_reconfigure
)

# Messages, services and actions

# Dynamic reconfigure parameters

# Catkin specific configuration

catkin_package( CATKIN_DEPENDS
  roscpp rospy
  std_msgs
  geometry_msgs
  message_runtime
  message_filters
)

# Build

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(local_planner src/local_planner.cpp)
target_link_libraries(local_planner ${catkin_LIBRARIES})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)