cmake_minimum_required(VERSION 3.0.2)
project(usart_port)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
actionlib
urdf
cmake_modules
control_msgs
control_toolbox
controller_manager
hardware_interface
joint_limits_interface
roscpp
sensor_msgs
std_msgs
trajectory_msgs
transmission_interface
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usart_port
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include ${catkin_INCLUDE_DIRS}
  $ENV{HOME}/robot_ws/devel/include

  ${catkin_INCLUDE_DIRS}
)

add_executable(usart_send src/publish_node.cpp 
			    src/linux_serial.cpp)
target_link_libraries(usart_send ${catkin_LIBRARIES})

add_executable(robot_hardware_interface src/robot_hardware_interface.cpp)
target_link_libraries(robot_hardware_interface ${catkin_LIBRARIES})