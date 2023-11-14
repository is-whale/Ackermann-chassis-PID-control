cmake_minimum_required(VERSION 3.0.2)
project(ackermann_pid_pursuit)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf2
)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ackermann_pid_pursuit
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs
)
include_directories(${CMAKE_INSTALL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  include
)
add_executable(${PROJECT_NAME} src/ackermann_pid_pursuit.cpp src/judge_direction.cpp src/pid_lib.cpp) # Removed the duplicate call to add_executable() here.
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} glog) # Moved this line here after adding the missing target linking library.
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}) # Removed the duplicate install() call here. Installation should be done only once for all targets. Also moved this line here.
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.hpp") # Removed the duplicate install() call here. Installation should be done only once for all targets. Also moved this line here.
