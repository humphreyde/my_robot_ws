cmake_minimum_required(VERSION 3.0.2)
project(robot_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  arbotix_msgs
  control_msgs
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  $ENV{HOME}/baxter_ws/devel/include
  include 
  $ENV{HOME}/robot_ws/devel/include
  ${catkin_INCLUDE_DIRS} 
  "/usr/include/eigen3"
)
 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_planning
#  CATKIN_DEPENDS roscpp std_msg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_arm_test
  src/robot_arm_test.cpp
  src/robot_planning.cpp
  src/robot_kinematics_solver.cpp
)
add_dependencies(robot_arm_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_arm_test
  ${catkin_LIBRARIES}
)
