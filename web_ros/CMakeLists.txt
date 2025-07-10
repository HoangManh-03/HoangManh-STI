cmake_minimum_required(VERSION 3.0.2)
project(web_ros)

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
)
catkin_package(

)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pub_velocity src/pub_velocity.cpp)

# Liên kết các thư viện ROS cần thiết cho node
target_link_libraries(pub_velocity
  ${catkin_LIBRARIES}
)
