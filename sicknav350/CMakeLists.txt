cmake_minimum_required(VERSION 2.8.3)
project(sicknav350)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs sicktoolbox rosconsole diagnostic_updater tf message_generation message_pkg)

## System dependencies are found with CMake's conventions
#find_package(Threads)

generate_messages(
   DEPENDENCIES
   std_msgs
   message_pkg
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs sicktoolbox rosconsole diagnostic_updater tf message_runtime move_base_msgs actionlib
	DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

add_executable(sicknav350_node src/sicknav350_node.cpp)
target_link_libraries(sicknav350_node ${catkin_LIBRARIES})
add_dependencies(sicknav350_node sicknav350_gencpp)

add_executable(sicknav_node src/sicknav_node.cpp)
target_link_libraries(sicknav_node ${catkin_LIBRARIES})
add_dependencies(sicknav_node sicknav_gencpp)

add_executable(sicknav350_oldNode src/sicknav350_oldNode.cpp)
target_link_libraries(sicknav350_oldNode ${catkin_LIBRARIES})
add_dependencies(sicknav350_oldNode sicknav350_old_gencpp)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS sicknav350_node sicknav_node sicknav350_oldNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# -- 
add_executable(robotPose_nav src/robotPose_NavFix.cpp)
target_link_libraries(robotPose_nav ${catkin_LIBRARIES})

add_executable(datest src/datest.cpp)
target_link_libraries(datest ${catkin_LIBRARIES})

install(TARGETS robotPose_nav
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# -- 
# add_executable(robotPose_navfix src/robotPose_NavFix.cpp)
# target_link_libraries(robotPose_navfix ${catkin_LIBRARIES})

# install(TARGETS robotPose_navfix
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# -- 
add_executable(sicknav350_nodeReflector src/sicknav350_nodeReflector.cpp)
target_link_libraries(sicknav350_nodeReflector ${catkin_LIBRARIES})

install(TARGETS sicknav350_nodeReflector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  
