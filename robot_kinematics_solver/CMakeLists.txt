cmake_minimum_required(VERSION 3.0.2)
project(robot_kinematics_solver)

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
  tf
)

include_directories(
  include ${catkin_INCLUDE_DIRS} "/usr/include/eigen3" )

  
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_kinematics_solver
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(test_api_node
  src/test_api_node.cpp
  src/robot_kinematics_solver.cpp
)
add_dependencies(test_api_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_api_node
  ${catkin_LIBRARIES}
)
