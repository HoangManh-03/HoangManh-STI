cmake_minimum_required(VERSION 3.0.2)
project(testMotionPlanning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sti_msgs
  nav_msgs
  message_pkg
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES testMotionPlanning
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

add_executable(goal_control src/MainMotionControl.cpp src/goalControl.cpp src/pidControl.cpp src/move_task.cpp src/bezier_curves.cpp)
target_link_libraries(goal_control ${catkin_LIBRARIES})