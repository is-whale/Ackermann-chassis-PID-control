cmake_minimum_required(VERSION 3.0.2)
project(ackermann_pid_pursuit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf2
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ackermann_pid_pursuit
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs
)
###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${CMAKE_INSTALL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  include
  )

## Declare a C++ executable
add_executable(${PROJECT_NAME}
src/ackermann_pid_pursuit.cpp
src/judge_direction.cpp
src/pid_lib.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   glog
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

