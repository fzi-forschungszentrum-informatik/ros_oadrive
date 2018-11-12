// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 *
 * \author  Kolja Esders <esders@fzi.de>
 * \date    2017
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 * 
 */
//----------------------------------------------------------------------

#ifndef ROS_OADRIVE_TRAJECTORY_GENERATOR_H_
#define ROS_OADRIVE_TRAJECTORY_GENERATOR_H_

#include <oadrive_core/Trajectory2d.h>
#include <oadrive_world/Environment.h>
#include <oadrive_world/MultiTrajectory.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <oadrive_world/StateMachine.h>
#include <oadrive_lanedetection/RoadPatching/Map.h>
#include <oadrive_util/Config.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>
#include <ros_oadrive/Event.h>
#include <ros_oadrive/MultiTrajectory.h>
#include <ros_oadrive/StreetPatch.h>
#include <ros_oadrive/StreetPatchList.h>
#include <ros_oadrive/TrackedObjects.h>
#include <ros_oadrive/ControllerStatus.h>
#include "ros_oadrive/MarkerPosition.h"
#include "ros_oadrive/StateChange.h"
#include <ros_oadrive/Ultrasonic.h>
#include <ros_oadrive/StreetPatchOccupancy.h>
#include <unordered_set>
#include <chrono>


#define MEASURE_EXEC_TIME

namespace ros_oadrive {

/*!
  \class TrajectoryGenerator
  \brief Generates and publishes trajectories.

  The trajectory generation is based on the current car pose as well as the
  street patches that have been generated.

  Publishes:
    * /aadc/planning/trajectory -> MultiTrajectory

  Subscribes:
    * /aadc/marker_position
    * /aadc/perception/patch
    * /aadc/planning/event
*/
class PlanningNode {
 public:
  //! Length in meters of the length expansion of the junction patch towards the vehicle. This is
  //! uses to create an event region that is extended in the direction of the vehicle to trigger
  //! blinking etc. before driving on the actual junction.
  const double JUNCTION_EXPANSION_LENGTH_IN_METERS = 1.0f;
  const double JUNCTION_HALT_EXPANSION_LENGTH_IN_METERS = 0.8f;

  /*!
    \brief Handles initial publish & subscribe.
   */
  PlanningNode(ros::NodeHandle node_handle, oadrive::util::CoordinateConverter *converter);

  /*!
    \brief Handles new car poses from subscription.
   */
  void poseCallback(const ros_oadrive::MarkerPositionConstPtr &pose_msg);

  /*!
    \brief Handles new street patches from subscription.
   */
  void patchCallback(const StreetPatchList::ConstPtr &patches_msg);

  /*!
    *
    */
  void trackedObjectCallback(const TrackedObjects::ConstPtr &trackedObjects);

  void controllerCallback(const ControllerStatus::ConstPtr& msg);

  void bypassTimerCallback(const ros::TimerEvent& event);

  void stateCallback(const StateChange::ConstPtr& msg);

  void usCallback(const ros_oadrive::Ultrasonic::ConstPtr& US_msg);

  void eventCallback(const ros_oadrive::Event::ConstPtr& msg);

  void mapCallback(const std_msgs::String::ConstPtr &msg);

  void patchOccupancyCallback(const ros_oadrive::StreetPatchOccupancy::ConstPtr &msg);


  /*!
    \brief Generates and returns an exemplary trajectory.
    \return an exemplary trajectory
   */
  static MultiTrajectory buildExemplaryTrajectory();

 private:
  /*!
    \brief Advertise topics that will be published.
   */
  void advertise();

  /*!
    \brief Subscribe to necessary topics.
   */
  void subscribe();

  //! Resets node so that we can restart a sector.
  void reset();

  // generates a driving command
  oadrive::world::DrivingCommand generateLongitudinalDrivingCommand();

  bool mergingCheckForStop();

  void publishDrivingCommand(oadrive::world::DrivingCommand drivingCommand);

  /*!
    \brief Publish the current trajectory as marker message for rviz.
  */

  void publishStreetVisualization(oadrive::world::PatchPtrList street);

  void showDebugImage();

  double distanceToPatch(oadrive::world::PatchPtr patch);

  void publishJunctionEventRegions();

  void enableSlowDriving() {
    std_msgs::Float64 speedMsg;
    speedMsg.data = Config::getDouble("Driver", "SlowSpeed", 0.5);
    m_speed_publisher.publish(speedMsg);
    m_slow_driving = true;
  }

  void enableSlowParkDriving() {
    std_msgs::Float64 speedMsg;
    speedMsg.data = Config::getDouble("Driver", "SlowParkSpeed", 0.3);
    m_speed_publisher.publish(speedMsg);
    m_slow_driving = true;
  }

  void disableSlowDriving() {
    std_msgs::Float64 speedMsg;
    speedMsg.data = Config::getDouble("Driver", "NormalSpeed", 1.25);
    m_speed_publisher.publish(speedMsg);
    m_slow_driving = false;
  }

  int getLaneLevel(LaneType lane) {
    switch (lane) {
      case LANE_CENTER:
      return 1;

      case LANE_LEFT:
      return 2;

      default:
      return 0;
    }
  }

  /*!
    \brief Populates the header of the given message with necessary information.
    \param msg Message whose header will be populated
   */
  void populateMessageHeader(MultiTrajectory &msg);

  ros::NodeHandle m_node;

  ros::Subscriber m_event_subscriber;

  ros::Subscriber m_patch_subscriber;

  ros::Subscriber m_pose_subscriber;

  ros::Subscriber m_trackedObject_subscriber;

  ros::Subscriber m_controller_subscriber;

  ros::Subscriber m_state_subscriber;

  ros::Subscriber m_us_subscriber;

  ros::Subscriber m_map_subscriber;

  ros::Subscriber m_patch_occupancy_subscriber;

  //! Publisher for the /aadc/planning/trajectory topic.
  ros::Publisher m_trajectory_publisher;

  ros::Publisher m_rviz_street_publisher;

  ros::Publisher m_event_region_publisher;

  ros::Publisher m_longitudinalCommand_publisher;

  ros::Publisher m_event_publisher;
  ros::Publisher m_speed_publisher;

  ros::Timer mBypassSpeedTimer;

  oadrive::util::CoordinateConverter *mConverter;

#ifdef MEASURE_EXEC_TIME
  double mStartTime = 0.0;
  double mExecTimeAddPatches = 0.0;
  double mExecTimeGenTraj = 0.0;
#endif

  std::unordered_set<uint64_t> m_entered_junction_regions;

  // Map
  tf::TransformListener m_tf_listener;
  bool m_slow_driving = false;
  bool m_slow_intersection_driving = false;
  bool m_slow_parking_driving = false;
  bool m_slow_bypass_driving = false;
  oadrive::lanedetection::Map m_map;

  // last Driving Command
  ros_oadrive::Ultrasonic m_last_us;
  std::chrono::time_point<std::chrono::system_clock> m_last_us_occupied;
  std::chrono::time_point<std::chrono::system_clock> m_merging_start;
  bool m_merging_done = false;
  bool m_cross_section_halt = false;

  oadrive::world::DrivingCommand m_last_command = oadrive::world::DrivingCommand::DRIVE;
  std::chrono::time_point<std::chrono::system_clock> m_last_stop
  ;
  std::chrono::time_point<std::chrono::system_clock> m_last_reset;
  bool m_reset_active = false;

  //! State for event/trajectory handling
  StateMachine::State m_current_state;
};

}  // namespace ros_oadrive

#endif /* ROS_OADRIVE_TRAJECTORY_GENERATOR_H_ */
