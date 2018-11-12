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

#ifndef ROS_OADRIVE_EVENT_TRIGGER_NODE_H_
#define ROS_OADRIVE_EVENT_TRIGGER_NODE_H_

#include <oadrive_obstacle/ObjectTracker.h>
#include <oadrive_world/Environment.h>
#include <ros/ros.h>
#include <ros_oadrive/Event.h>
#include <ros_oadrive/EventRegion.h>
#include <ros_oadrive/MarkerPosition.h>
#include <ros_oadrive/TrackedObjects.h>
#include <ros_oadrive/TrafficSign.h>
#include <ros_oadrive/JuryManeuver.h>
#include <ros_oadrive/StateChange.h>
#include <oadrive_world/StateMachine.h>
#include <std_msgs/String.h>
#include <unordered_map>
#include <unordered_set>
#include <tf/transform_listener.h>

namespace ros_oadrive {

struct ParkingSpace {
  int id;
  double x;
  double y;
  bool status;
  double direction;
};

/*!
  \class EventTriggerNode
  \brief Triggers events for entering and leaving event regions.

  Publishes:
    * /aadc/planning/event

  Subscribes:
    * /aadc/marker_position
    * /aadc/planning/event
    * /aadc/planning/event_region
    * /aadc/object_tracking/tracked_objects
*/
class EventTriggerNode {
 public:
  // Height treshold to distinguish children from normal persons
  const double CHILD_HEIGHT_THRESHOLD = 0.2f;
  //! Minimal distance in meters from the car to the center of the event region
  //! after which the event region is discarded (value is inclusive).
  const double DISCARD_MIN_DISTANCE = 4.f;

  //! Maximal distance in meters while an object is considered to be close to
  //! the car (value is inclusive).
  const double OBJECT_CLOSE_MAX_DISTANCE = 1.f;

  //! Margin for OBJECT_CLOSE_MAX_DISTANCE after which the object is
  //! regarded to be far away. This margin is needed to prevent a scenario where
  //! the car is suddenly close to the vehicle and then the position of the
  //! vehicle changes so that the car is now again regarded to be far away.
  const double OBJECT_CLOSE_MARGIN = OBJECT_CLOSE_MAX_DISTANCE + 0.3f;

  //! Upper bound on the distance between our current car position and the parking space
  //! that we have to target for parking so that we can be sure that we are at the correct parking
  //! area location in general.
  const double CORRECT_PARKING_AREA_THRESHOLD = 5.f;

  //! Distance between center point of the event region in front of our parking space and the take
  //! off point where the parking trajectory is started. The distance is parallel to the direction
  //! we are driving on the road.
  const double DISTANCE_TO_TAKE_OFF_POINT = 0.50f;

  /*!
    \brief Handles initial publish & subscribe.
   */
  EventTriggerNode(ros::NodeHandle node_handle);

  /*!
    \brief Handles new car poses from subscription.
   */
  void poseCallback(const ros_oadrive::MarkerPosition::ConstPtr &pose_msg);

  /*!
    \brief Handles new events from subscription.
   */
  void eventCallback(const Event::ConstPtr &event_msg);

  /*!
   \brief Handles new state transitions from subscription.
   */
  void stateCallback(const ros_oadrive::StateChange::ConstPtr &msg);

  /*!
    \brief Handles new event regions from subscription.
    \param
   */
  void eventRegionCallback(const EventRegion::ConstPtr &event_region_msg);

  /*!
    \brief Handles tracked objects from subscription.
   */
  void trackedObjectsCallback(const TrackedObjects::ConstPtr &tracked_objects_msg);

  /*!
    \brief Handles tracked objects from subscription.
   */
  void currentManeuverCallback(const ros_oadrive::JuryManeuver::ConstPtr &current_maneuver_msg);

  /*!
    \brief Resets this node so that it can be used at the start of a sector.
   */
  void reset();

 private:
  /*!
    \brief Advertise topics that will be published.
   */
  void advertise();

  /*!
    \brief Subscribe to the following topics:
      /aadc/marker_position
      /aadc/planning/event_region
   */
  void subscribe();

  std::string readFile(std::string file_path);

  void createParkingRegion(int parkingSpaceId);
  
  void readParkingSpacesConfig(std::string config_path);

  void handleTrackedObjects();

  void handleTrackedObjectEvents(const oadrive::obstacle::TrackedObject object);

  /*!
    \brief Helper method to build an event message from the given parameters.
    \param event_region Additional data for the event.
    \param entered_region Specifies whether the car just entered or left the
    region.
   */
  Event buildEventMessage(const oadrive::world::EventRegion &event_region, bool entered_region);

  Event buildEventMessage(const oadrive::obstacle::TrackedObject &tracked_object,
                          bool approached_object);

  /*!
    \brief Returns whether the given event region should be discard from the
    list of event regions we are maintaining. An event region should be
    discarded if it is no longer relevant for us. This could be the case if it
    is too far away from the current position for example. \param event_region
    to check
   */
  bool shouldBeDiscarded(const oadrive::world::EventRegion &event_region);

  /*!
    \brief Returns whether our car is currently inside of the given event
    region. \param event_region to check \return true if our car (ego) is in the
    given event region
   */
  bool isEgoInEventRegion(const oadrive::world::EventRegion &event_region);

  bool isEgoCloseToObject(const oadrive::obstacle::TrackedObject &object);

  bool isEgoInMarginOfObject(const oadrive::obstacle::TrackedObject &object);

  double distanceToObject(const oadrive::obstacle::TrackedObject &object);

  double degreeToRadian(double degree);

  ros::NodeHandle m_node;

  ros::Subscriber m_ego_pose_subscriber;

  ros::Subscriber m_event_subscriber;

  ros::Subscriber m_event_region_subscriber;

  ros::Subscriber m_tracked_objects_subscriber;

  ros::Subscriber m_current_maneuver_subscriber;

  ros::Subscriber m_state_subscriber;

  ros::Publisher m_event_publisher;

  ros::Publisher m_event_region_publisher;

  tf::TransformListener mTfListener;

  //! Stores current position of our car.
  oadrive::core::Pose2d m_ego_pose;

  //! Check whether we already received a pose from the car. Needed since
  //! m_ego_pose will have a default value of (0, 0, 0).
  bool m_pose_received;

  // Store if we just reported a sign
  bool m_parking_approached = false;
  bool m_zebra_approached = false;

  // Store if there was a child in the last detection msg
  bool m_child_detected = false;

  //! Identifier of the current maneuver
  std::string m_current_maneuver;
  int m_current_maneuver_extra = 0;

  //! List of event regions where we indicate for each event region whether the
  //! car is currently in it.
  std::unordered_map<uint64_t, std::pair<oadrive::world::EventRegion, bool>> m_event_regions;

  //! Set of event region UIDs that are currently being handled by this node.
  //std::unordered_set<uint64_t> m_event_region_uids;

  oadrive::obstacle::TrackedObjects m_tracked_objects;

  //! Distances from last update
  std::unordered_map<uint64_t, double> m_object_distance;

  //! Stores whether the object with the given id was already close to the car
  //! at one point in time.
  std::unordered_map<uint64_t, bool> m_object_was_close;

  //! Stores IDs of objets that are no longer relevant since they already
  //! triggered events.
  std::unordered_set<uint64_t> m_processed_objects;

  //! Stores parking spaces on the map. Key is the id of the parking space.
  std::unordered_map<int, ParkingSpace> m_parking_spaces;

  StateMachine::State mCurrentState;
};

}  // namespace ros_oadrive

#endif /* ROS_OADRIVE_TRAJECTORY_GENERATOR_H_ */
