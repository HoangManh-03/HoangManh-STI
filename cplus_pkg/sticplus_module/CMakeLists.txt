cmake_minimum_required(VERSION 2.8.3)
project(sticplus_module)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  rospy 
  tf
  tf2_ros
  dynamic_reconfigure
  sti_msgs
  nav_msgs
  message_pkg
  sensor_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  sti_msgs
  message_pkg

)

catkin_package(

)

# add_service_files(FILES AddTwoInts.srv)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch")

# install(DIRECTORY data/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

add_executable(Parkingnode_cpp src/parking_nav_nm.cpp)
target_link_libraries(Parkingnode_cpp ${catkin_LIBRARIES})
add_dependencies(Parkingnode_cpp Parkingnode_generate_messages_cpp)

add_executable(goalControlnode_cpp src/goal_control.cpp)
target_link_libraries(goalControlnode_cpp ${catkin_LIBRARIES})
add_dependencies(goalControlnode_cpp goalcontrol_generate_messages_cpp)