cmake_minimum_required(VERSION 2.8.3)
project(robot_webots_api)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  ubt_core_msgs
  message_generation
  camera_info_manager
)



## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SceneSelection.srv
 )


# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES walkerx_webots_api
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")

add_definitions("-std=c++11")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
lib
  ${catkin_LIB_DIRS}
)

 add_executable(${PROJECT_NAME}
     src/webots.cpp
     src/basic_function.cpp
     src/position_random.cpp
#     src/avoid_jointlimit.cpp
#     src/walker_iksolver.cpp
 )

add_executable(webots_test_node
    src/position_random.cpp
    src/test.cpp
)

add_executable(camera
src/basic_function.cpp
src/extra_camera.cpp
)


 add_dependencies(${PROJECT_NAME}
     ${${PROJECT_NAME}_EXPORTED_TARGETS}
     ${catkin_EXPORTED_TARGETS})

 add_dependencies(webots_test_node
     ${${PROJECT_NAME}_EXPORTED_TARGETS}
     ${catkin_EXPORTED_TARGETS})

 add_dependencies(camera
     ${${PROJECT_NAME}_EXPORTED_TARGETS}
     ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   Controller
 )
target_link_libraries(webots_test_node
  ${catkin_LIBRARIES}
  Controller
##  walker_arm_ik
)

target_link_libraries(camera
  ${catkin_LIBRARIES}
  Controller
)


install(TARGETS ${PROJECT_NAME} webots_test_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
