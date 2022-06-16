cmake_minimum_required(VERSION 2.8.3)
project(unl_smart_robotic_home)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    pluginlib
    #geometric_shapes
    pcl_ros
    pcl_conversions    
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roscpp
    rospy
    darknet_ros_msgs
    cv_bridge    
    std_msgs
    message_generation
)

#find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS include)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

### Generate services in the 'srv' folder
add_service_files(#DIRECTORY srv
  FILES
   node_ready.srv
   move_absolute.srv
   move_relative.srv
   execute_plan.srv
   elevator_door_request.srv
   elevator_car_request.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

### Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)


catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    interactive_markers
    message_runtime
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_LIBRARY_DIRS}  
  ${PCL_LIBRARY_DIRS}
)
include_directories(${catkin_INCLUDE_DIRS} include/Scheduler)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

#If boost is needed, add  ${Boost_LIBRARIES}  to the target_link_libraries

# move_door
add_executable(move_door src/move_door.cpp)
target_link_libraries(move_door ${catkin_LIBRARIES})
install(TARGETS move_door DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# swing_door
add_executable(swing_door src/swing_door.cpp)
target_link_libraries(swing_door ${catkin_LIBRARIES})
install(TARGETS swing_door DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# move_base_director
add_executable(move_base_director src/move_base_director.cpp)
target_link_libraries(move_base_director ${catkin_LIBRARIES})
install(TARGETS move_base_director DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# bot_manager
# NOTE: Added the stdc++fs to the link libraries, even though it is
#       technically part of the spec since we're using C++ 17. But
#       gcc (at least, my version) is still catching up.
add_executable(bot_manager
  src/bot_manager.cpp
  src/inspection_plan.cpp
  src/inventoryClerk.cpp
  src/cvUtils.cpp
  src/segmentation_pipeline.cpp
  src/convexHull.cpp
  src/gaussKernel.cpp
  src/map_writer.cpp
)
target_link_libraries(bot_manager ${catkin_LIBRARIES} stdc++fs)
install(TARGETS bot_manager DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# segmenter
add_executable(segmenter
  src/segmenter.cpp
  src/segmentation_pipeline.cpp
  src/convexHull.cpp
  src/gaussKernel.cpp
)
target_link_libraries(segmenter
  ${catkin_LIBRARIES}
  pcl_visualization
)
install(TARGETS segmenter DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# move_elevator_door
#add_executable(move_elevator_door src/move_elevator_door.cpp)
#target_link_libraries(move_elevator_door ${catkin_LIBRARIES})
#install(TARGETS move_elevator_door DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# move_elevator_car
#add_executable(move_elevator_car src/move_elevator_car.cpp)
#target_link_libraries(move_elevator_car ${catkin_LIBRARIES})
#install(TARGETS move_elevator_car DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# NOTE: these are also included in the ElevatorWorld project, so comment out
# from here to avoid duplication (or uncomment when no longer using elevator world)
#
# elevator_door_manager
#add_executable(elevator_door_manager src/services/ElevatorDoorManager.cpp)
#target_link_libraries(elevator_door_manager ${catkin_LIBRARIES})
#install(TARGETS elevator_door_manager DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# elevator_car_manager
#add_executable(elevator_car_manager src/services/ElevatorCarManager.cpp)
#target_link_libraries(elevator_car_manager ${catkin_LIBRARIES})
#install(TARGETS elevator_car_manager DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
