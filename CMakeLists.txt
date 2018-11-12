# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------

cmake_minimum_required(VERSION 2.8.3)
project(ros_oadrive)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
find_package(icmaker REQUIRED) # for additional find-scripts
find_package(icl_core REQUIRED)
find_package(oadrive REQUIRED)
find_package(EigenAlways REQUIRED)
find_package(OpenCV REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(aruco REQUIRED )


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
## catkin_package(CATKIN_DEPENDS message_runtime )
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MarkerPosition.msg
  ExternalMarkerPosition.msg
  ControllerStatus.msg
  ControllerCommand.msg
  EmergencyBrake.msg
  ExtendedPose2d.msg
  MultiTrajectory.msg
  Trajectory2d.msg
  StreetPatch.msg
  StreetPatchList.msg
  Event.msg
  EventRegion.msg
  StateChange.msg
  DetectedObject.msg
  DetectedCar.msg
  DetectedPerson.msg
  DetectedObjects.msg
  TrackedObject.msg
  TrackedCar.msg
  TrackedPerson.msg
  TrackedObjects.msg
  LightStatus.msg
  EnvironmentObject.msg
  TrafficSign.msg
  TrafficSignToBackend.msg
  ImagePosition.msg
  Ultrasonic.msg
  Pose2DStamped.msg
  JuryManeuver.msg
  StreetPatchOccupancy.msg
  NavObstruction.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   EvokeTransition.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  CATKIN_DEPENDS message_runtime cv_bridge image_transport sensor_msgs std_msgs geometry_msgs
# INCLUDE_DIRS include
# LIBRARIES ros_oadrive
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${icl_core_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Add definitions derived from dependencies
add_definitions(
  ${icl_core_DEFINITIONS}
  ${oadrive_DEFINITIONS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ros_oadrive.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ControllerNode
  src/ControllerNode.cpp
  src/converter/TrackedObjectsConverter.cpp
  src/converter/TrackedObjectConverter.cpp
  src/converter/TrafficSignConverter.cpp
  src/converter/EnvironmentObjectConverter.cpp
  src/converter/Pose2dConverter.cpp
)
add_executable(RosRawDataRecorder
  src/debug_nodes/RosRawDataRecorder.cpp
  src/converter/Pose2dConverter.cpp
)
add_executable(TrajectoryRecorder
  src/debug_nodes/TrajectoryRecorder.cpp
)
add_executable(TrajectoryVisualizationNode
  src/debug_nodes/TrajectoryVisualizationNode.cpp
)
add_executable(ObjectVisualizationNode
  src/debug_nodes/ObjectVisualizationNode.cpp
  src/converter/Pose2dConverter.cpp
  src/converter/TrafficSignConverter.cpp
  src/converter/TrackedObjectConverter.cpp
  src/converter/TrackedObjectsConverter.cpp
  src/converter/EnvironmentObjectConverter.cpp
)
add_executable(UltrasonicVisualizationNode
  src/debug_nodes/UltrasonicVisualizationNode.cpp
)
add_executable(PositionFakerNode
  src/debug_nodes/PositionFakerNode.cpp
)
add_executable(rosDummyTrajectory
  src/debug_nodes/rosDummyTrajectory.cpp
)
add_executable(XboxControllerNode
  src/debug_nodes/XboxControllerNode.cpp
  src/debug_nodes/libgamepad/gamepad.c
)
add_executable(TeleOperationControllerNode
  src/TeleOperationControllerNode.cpp
  src/icl_hardware_g25_wheel/G25RacingWheel.cpp
  src/icl_hardware_g25_wheel/Logging.cpp
)
add_executable(TeleOperationSwitchNode
  src/TeleOperationSwitchNode.cpp
)
add_executable(MapEventNode
  src/MapEventNode.cpp
)
add_executable(NavigatorNode
  src/NavigatorNode.cpp
  src/converter/TrackedObjectsConverter.cpp
  src/converter/TrackedObjectConverter.cpp
)
add_executable(PlanningNode
  src/PlanningNode.cpp
  src/converter/MultiTrajectoryConverter.cpp
)
add_executable(RoadPerceptionNode
  src/RoadPerceptionNode.cpp
)
add_executable(EventTriggerNode
  src/EventTriggerNode.cpp
  src/converter/EventRegionConverter.cpp
)
add_executable(StateMachineNode
  src/StateMachineNode.cpp
  src/converter/EventRegionConverter.cpp
)
add_executable(LightControlNode
  src/LightControlNode.cpp
)
add_executable(ObjectDetectorNode
  src/ObjectDetectorNode.cpp
)

add_executable(ObjectTrackerNode
  src/ObjectTrackerNode.cpp
)

add_library(ObjectConversion
  src/converter/StreetPatchConverter.cpp
  src/converter/DetectedObjectsConverter.cpp
  src/converter/TrackedObjectsConverter.cpp
  src/converter/TrackedObjectConverter.cpp
  src/converter/TrafficSignConverter.cpp
  src/converter/EnvironmentObjectConverter.cpp
  src/converter/EventRegionConverter.cpp
  src/converter/Pose2dConverter.cpp
)
add_executable(BirdviewConverterNode
  src/BirdviewConverterNode.cpp
)
add_executable(EmergencyBrakeNode
  src/EmergencyBrakeNode.cpp
)

add_executable(SirenDetectorNode
  src/SirenDetectorNode.cpp
)

add_dependencies(rosDummyTrajectory           ${PROJECT_NAME}_gencpp)
add_dependencies(RosRawDataRecorder             ${PROJECT_NAME}_gencpp)
add_dependencies(TrajectoryRecorder ${PROJECT_NAME}_gencpp)
add_dependencies(XboxControllerNode ${PROJECT_NAME}_gencpp)
add_dependencies(TeleOperationSwitchNode ${PROJECT_NAME}_gencpp)
add_dependencies(TeleOperationControllerNode ${PROJECT_NAME}_gencpp)
add_dependencies(ObjectDetectorNode ${PROJECT_NAME}_gencpp)


add_dependencies(ObjectDetectorNode darknet_ros_msgs_gencpp)
add_dependencies(ObjectConversion darknet_ros_msgs_gencpp)
add_dependencies(ObjectConversion ${PROJECT_NAME}_gencpp)

add_dependencies(StateMachineNode             ${PROJECT_NAME}_gencpp)
add_dependencies(LightControlNode             ${PROJECT_NAME}_gencpp)

add_dependencies(RoadPerceptionNode           ${PROJECT_NAME}_gencpp)
add_dependencies(EventTriggerNode             ${PROJECT_NAME}_gencpp)

add_dependencies(BirdviewConverterNode        ${PROJECT_NAME}_gencpp)
add_dependencies(MapEventNode                 ${PROJECT_NAME}_gencpp)
add_dependencies(NavigatorNode                 ${PROJECT_NAME}_gencpp)
add_dependencies(PlanningNode                 ${PROJECT_NAME}_gencpp)
add_dependencies(TrajectoryVisualizationNode      ${PROJECT_NAME}_gencpp)
add_dependencies(ObjectVisualizationNode      ${PROJECT_NAME}_gencpp)
add_dependencies(UltrasonicVisualizationNode      ${PROJECT_NAME}_gencpp)
add_dependencies(PositionFakerNode      ${PROJECT_NAME}_gencpp)
add_dependencies(ControllerNode               ${PROJECT_NAME}_gencpp)
add_dependencies(EmergencyBrakeNode           ${PROJECT_NAME}_gencpp)

add_dependencies(SirenDetectorNode ${PROJECT_NAME}_gencpp)



target_link_libraries(ObjectConversion
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(ControllerNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
)
target_link_libraries(RosRawDataRecorder
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(TrajectoryRecorder
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(TrajectoryVisualizationNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(ObjectVisualizationNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(UltrasonicVisualizationNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(PositionFakerNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
)
target_link_libraries(rosDummyTrajectory
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
)
target_link_libraries(XboxControllerNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  udev
)
target_link_libraries(TeleOperationSwitchNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
)
target_link_libraries(TeleOperationControllerNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  udev
)
target_link_libraries(ObjectDetectorNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ObjectConversion
)

target_link_libraries(ObjectTrackerNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ObjectConversion
)
target_link_libraries(BirdviewConverterNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
)
target_link_libraries(StateMachineNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ObjectConversion
)
target_link_libraries(LightControlNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
)
target_link_libraries(PlanningNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ObjectConversion
)
target_link_libraries(MapEventNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(NavigatorNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ObjectConversion
)
target_link_libraries(EventTriggerNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ObjectConversion
)

target_link_libraries(RoadPerceptionNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ObjectConversion
)
target_link_libraries(EmergencyBrakeNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ObjectConversion
)

target_link_libraries(SirenDetectorNode
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ObjectDetectorNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS RoadPerceptionNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS MapEventNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS NavigatorNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS PlanningNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS EventTriggerNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS StateMachineNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS LightControlNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS BirdviewConverterNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install config files
install(DIRECTORY
  etc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
