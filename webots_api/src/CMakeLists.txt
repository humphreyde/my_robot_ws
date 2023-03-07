link_directories(
  ${CMAKE_SOURCE_DIR}
  ${catkin_LIB_DIRS}
  ../lib
  /usr/local/lib
#  /home/lzc/fcl_ws/usr/local/lib
#  /usr/lib/x86_64-linux-gnu
#/home/lzc/Downloads/fcl-master/build/lib
#  ../lib/x86_64-linux-gnu
#  /home/lzc/catkin_ws/src/walker_ik/lib
#  /home/lzc/fcl_ws/usr/local/lib/x86_64-linux-gnu
  )
#add_library(fcl SHARED IMPORTED )
#set_target_properties(fcl PROPERTIES IMPORTED_LOCATION ..lib/x86_64-linux-gnu/libfcl.so.0.6.0)
#add_library(walkerIK SHARED
#    ../include/walker_ik.h
#    walker_iksolver.cpp
#)
add_executable(${PROJECT_NAME}_walker_ik
  walker_ik.cpp
  walker_iksolver.cpp
  walker_collision_detection.cpp
  armq_opt.cpp
  optitrack_filter.cpp
  adjust_pose.cpp
#  /home/lzc/catkin_ws/src/walker_ik/lib/fcllib/src/geometry/shape/box.cpp
  avoid_jointlimit.cpp
)

#add_executable(${PROJECT_NAME}_test_ik
#  walker_iksolver.cpp
#  walker_collision_detection.cpp
#  test.cpp
#  armq_opt.cpp
#)

add_executable(${PROJECT_NAME}_walker_body
    walker_body_control.cpp
)

add_executable(${PROJECT_NAME}_vrtrix
  main.cpp
)
#target_link_libraries(${PROJECT_NAME}_walker_ik
#    ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_walker_ik
    ${catkin_LIBRARIES}  libfcl.so libccd.so  ${CCD_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} orocos-kdl)
set_target_properties(${PROJECT_NAME}_walker_ik PROPERTIES
                      OUTPUT_NAME walker_ik PREFIX "")

#target_link_libraries(${PROJECT_NAME}_test_ik
#  ${catkin_LIBRARIES} libfcl.so libccd.so)
###target_link_libraries(${PROJECT_NAME} orocos-kdl)
#set_target_properties(${PROJECT_NAME}_test_ik PROPERTIES
#                    OUTPUT_NAME test_ik PREFIX "")

target_link_libraries(${PROJECT_NAME}_walker_body
  ${catkin_LIBRARIES} )
#target_link_libraries(${PROJECT_NAME} orocos-kdl)
set_target_properties(${PROJECT_NAME}_walker_body PROPERTIES
                    OUTPUT_NAME walker_body PREFIX "")

target_link_libraries(${PROJECT_NAME}_vrtrix ${catkin_LIBRARIES}
  VRTRIXDataGloveLinuxDriver ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} udev)
#target_link_libraries(${PROJECT_NAME} orocos-kdl)
set_target_properties(${PROJECT_NAME}_vrtrix PROPERTIES
                    OUTPUT_NAME vrtrix PREFIX "")

#install(TARGETS
#    ${PROJECT_NAME}_optdata_to_walker
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
install(TARGETS 
  ${PROJECT_NAME}_walker_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

