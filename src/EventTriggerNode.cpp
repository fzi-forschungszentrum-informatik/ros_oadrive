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
 * \author  Kolja Esders <esders@fzi.de>
 * \date    2017
 * 
 * \author  Fabian Dürr
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

#include "ros_oadrive/EventTriggerNode.h"
#include "ros/ros.h"
#include "ros_oadrive/EventRegionConverter.h"
#include "ros_oadrive/MarkerPosition.h"
#include "ros_oadrive/Pose2dConverter.h"
#include "ros_oadrive/TrackedObjectConverter.h"
#include "ros_oadrive/TrackedObjectsConverter.h"

#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_core/Pose.h>
#include <oadrive_util/Config.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/smart_ptr.hpp>
#include "boost/assign.hpp"

using namespace boost;
using namespace boost::assign;
using namespace oadrive::world;

namespace ros_oadrive {

EventTriggerNode::EventTriggerNode(ros::NodeHandle node_handle)
    : m_node(node_handle) {
  advertise();
  subscribe();
  readParkingSpacesConfig(Config::getString("Map", "Path", ""));

  try {
    ROS_INFO("Waiting for transformations to be available..");
    mTfListener.waitForTransform("local", "world", ros::Time(0), ros::Duration(20.0));
    ROS_INFO("..Done!");
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }

  mCurrentState = StateMachine::State::INACTIVE;
}

std::string EventTriggerNode::readFile(std::string file_path) {
  std::ifstream file_stream(file_path);

  if (file_stream.is_open()) {
    std::stringstream ss;
    ss << file_stream.rdbuf();
    ROS_INFO_STREAM("Read file content from path: " << file_path);
    return ss.str();
  }

  ROS_WARN_STREAM("Unable to load road sign config from path: " << file_path);
  return "";
}

void EventTriggerNode::readParkingSpacesConfig(std::string config_path) {
  using boost::lexical_cast;  // for parsing string to int
  using boost::bad_lexical_cast;
  using boost::property_tree::ptree;

  std::string content = readFile(config_path);

  ptree pt;
  std::stringstream ss;
  ss << content;
  read_xml(ss, pt);

  BOOST_FOREACH (ptree::value_type const &v, pt.get_child("configuration")) {
    if (v.first == "parkingSpace") {
      ParkingSpace parking_space;

      try {
        parking_space.id = lexical_cast<int>(v.second.get_child("<xmlattr>.id").data());
        parking_space.x = lexical_cast<double>(v.second.get_child("<xmlattr>.x").data());
        parking_space.y = lexical_cast<double>(v.second.get_child("<xmlattr>.y").data());
        parking_space.status = lexical_cast<bool>(v.second.get_child("<xmlattr>.status").data());
        parking_space.direction =
            lexical_cast<double>(v.second.get_child("<xmlattr>.direction").data());
      } catch (const bad_lexical_cast &e) {
        ROS_INFO_STREAM("WARNING: Could not parse road sign attribute.");
      } catch (std::exception &e) {
        ROS_INFO_STREAM("WARNING: Could not parse road sign attribute.");
      }

      m_parking_spaces.insert(std::make_pair(parking_space.id, parking_space));
    }
  }
}

void EventTriggerNode::poseCallback(const ros_oadrive::MarkerPosition::ConstPtr &pose_msg) {
  m_ego_pose = Pose2dConverter::fromMessage(pose_msg->pose);
  m_pose_received = true;   

  handleTrackedObjects();

  // Check which events to trigger and which regions to discard.
  std::unordered_map<uint64_t, std::pair<oadrive::world::EventRegion, bool>>::iterator it = m_event_regions.begin();
  while (it != m_event_regions.end()) {
    oadrive::world::EventRegion region = (it->second).first;
    bool last_time_in_region = (it->second).second;
    bool currently_in_region = isEgoInEventRegion(region);

    // Publish event if we just entered or left a region
    if (currently_in_region != last_time_in_region) {
      if (currently_in_region) {
        ROS_INFO_STREAM("ENTERED event region [UID = " << region.getId() << "]");
      } else {
        ROS_INFO_STREAM("LEFT event region [UID = " << region.getId() << "]");
      }
      (it->second).second = currently_in_region;
      m_event_publisher.publish(buildEventMessage(region, currently_in_region));

      if (!currently_in_region && region.getOneTime()) {
        // we left a onetime region, so we drop it
        it = m_event_regions.erase(it);
        continue;
      }
    }
    // If we are too far away from the center of a region, we should discard it.
    // TODO: Should we also discard once we've left event region or allow for
    // re-entry?
    if (shouldBeDiscarded(region)) {
      it = m_event_regions.erase(it);
    } else {
      ++it;
    }
  }
}

void EventTriggerNode::stateCallback(const ros_oadrive::StateChange::ConstPtr &msg) {

  auto prevState = static_cast<StateMachine::State>(msg->prevState);
  auto newState = static_cast<StateMachine::State>(msg->newState);

  mCurrentState = newState;

  if (prevState == StateMachine::State::ACTIVE || prevState == StateMachine::State::INACTIVE || prevState == StateMachine::State::PARKING_LEFT || prevState == StateMachine::State::PARKING_RIGHT) {
    reset();
  }
}


void EventTriggerNode::eventCallback(const Event::ConstPtr &event_msg) {

  if (event_msg->type == "RESET") {
    reset();
  }
  
}

void EventTriggerNode::createParkingRegion(int parkingSpaceId) {
  ParkingSpace parking_space = m_parking_spaces[parkingSpaceId];

  // convert from map to odometry coordinates using tf
  geometry_msgs::PoseStamped worldPose;
  worldPose.header.frame_id = "world";
  worldPose.header.stamp = ros::Time::now();
  worldPose.pose.position.x = parking_space.x;
  worldPose.pose.position.y = parking_space.y;
  worldPose.pose.orientation = tf::createQuaternionMsgFromYaw(degreeToRadian(parking_space.direction));
  
  geometry_msgs::PoseStamped localPose;
  mTfListener.transformPose("local", ros::Time(0), worldPose, "world", localPose);

  tf::Quaternion q(
    localPose.pose.orientation.x,
    localPose.pose.orientation.y,
    localPose.pose.orientation.z,
    localPose.pose.orientation.w
  );
            
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  parking_space.x = localPose.pose.position.x;
  parking_space.y = localPose.pose.position.y;
  double dx = parking_space.x - m_ego_pose.translation()[0];
  double dy = parking_space.y - m_ego_pose.translation()[1];
  
  double parking_space_heading = yaw;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < CORRECT_PARKING_AREA_THRESHOLD) {
    oadrive::core::Pose2d center_pose;
    oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromOrientationRPY(
        center_pose, parking_space_heading + M_PI / 2.0f);
    center_pose.translation()[0] = parking_space.x + std::cos(parking_space_heading) * 0.25f;
    center_pose.translation()[1] = parking_space.y + std::sin(parking_space_heading) * 0.25f;

    // Generate "payload" for the event itself. We provide the take off point where the car starts
    // the parking trajectory.
    oadrive::core::Pose2d take_off_pose;
    oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromOrientationRPY(
        take_off_pose, parking_space_heading - M_PI / 2.0f);
      
    // const float DIST_Y = 0.66;
    // const float DIST_X = 0.25;
    const float DIST_Y = 0.66;
    const float DIST_X = 0.20;
    
    take_off_pose.translation()[0] =
        parking_space.x + std::cos(parking_space_heading - M_PI / 2.0f) * DIST_Y - std::sin(parking_space_heading - M_PI / 2.0f) * DIST_X;

    take_off_pose.translation()[1] =
        parking_space.y + std::sin(parking_space_heading - M_PI / 2.0f) * DIST_Y + std::cos(parking_space_heading - M_PI / 2.0f) * DIST_X;

    // Orientation is same as parking space (pointing towards street) so we need to increase width
    // in order to trigger the related "entering" event earlier.
    double width = 2.0f;
    double length = 0.50f;

    oadrive::world::EventRegion event_region(PARKING_SPACE_REGION, center_pose, take_off_pose, width, length);
    // TODO: Change this! Currently we need to make sure that the IDs of event regions are unique.
    // This should be solved differently.
    event_region.setId(100000 + parking_space.id);

    EventRegion event_region_msg = EventRegionConverter::toMessage(event_region);

    m_event_region_publisher.publish(event_region_msg);
  } else {
    ROS_WARN_STREAM("Parking to far " << dist << "!");
  }
}

void EventTriggerNode::eventRegionCallback(const EventRegion::ConstPtr &event_region_msg) {
  oadrive::world::EventRegion region = EventRegionConverter::fromMessage(*event_region_msg);

  bool currently_in_region = isEgoInEventRegion(region);

  if (m_event_regions.find(region.getId()) == m_event_regions.end()) {
    ROS_INFO_STREAM("RECEIVED NEW event region [UID = " << region.getId() << "]");
    if (currently_in_region) {
      ROS_INFO_STREAM("INSIDE NEW event region [UID = " << region.getId() << "]");

      ROS_INFO_STREAM("ENTERED event region [UID = " << region.getId() << "]");
      m_event_publisher.publish(buildEventMessage(region, currently_in_region));
    } else {
      ROS_INFO_STREAM("OUTSIDE NEW event region [UID = " << region.getId() << "]");
    }

    m_event_regions.emplace(std::make_pair(region.getId(), std::make_pair(region, currently_in_region)));
  
  } else {
    ROS_INFO_STREAM("RECEIVED EXISTING event region [UID = " << region.getId() << "]");
    if (!m_event_regions.at(region.getId()).second) {
      (m_event_regions.at(region.getId()).first).update(region);

      if (currently_in_region) {
        ROS_INFO_STREAM("INSIDE NEW event region [UID = " << region.getId() << "]");

        ROS_INFO_STREAM("ENTERED event region [UID = " << region.getId() << "]");

        m_event_publisher.publish(buildEventMessage(region, currently_in_region));

        m_event_regions.at(region.getId()).second = true;
      }
    }
  }
}

void EventTriggerNode::trackedObjectsCallback(const TrackedObjects::ConstPtr &tracked_objects_msg) {

  m_tracked_objects = TrackedObjectsConverter::fromMessage(*tracked_objects_msg);
  handleTrackedObjects();
}

void EventTriggerNode::currentManeuverCallback(const ros_oadrive::JuryManeuver::ConstPtr &msg) {
  m_current_maneuver = msg->action;
  m_current_maneuver_extra = msg->extra;
}

void EventTriggerNode::handleTrackedObjects() {
  if (!m_pose_received) {
    return;
  }

  for (auto &car : m_tracked_objects.cars) {
    handleTrackedObjectEvents(car.object);
  }

  bool personNearby = false;

  for (auto &person : m_tracked_objects.persons) {
    handleTrackedObjectEvents(person.object);

    if (person.object.calcDistTo(m_ego_pose) < 3.0) {
      personNearby = true;
    }
  }

  // Check for close parking sign:
  bool foundParking = false;
  if (m_current_maneuver == "cross_parking") {
    for (auto &sign: m_tracked_objects.trafficSigns) {
      if (sign.getType() != TrafficSignType::PARKING_AREA)
        continue;
      auto dist = sign.calcDistTo(m_ego_pose);

      if (sign.getType() == PARKING_AREA) {
        foundParking = true;
        if (dist <= 1.0) {
          if (!m_parking_approached) {
            ROS_INFO_STREAM("PUBLISHED APPROACHING PARKING SPACES");
            ros_oadrive::Event msg;
            msg.type = "APPROACHING_PARKING_SPACES";
            m_event_publisher.publish(msg);

            m_parking_approached = true;
          }
        
          // update event region
          int parking_space_id = m_current_maneuver_extra;

          if (m_parking_spaces.find(parking_space_id) == m_parking_spaces.end()) {
            ROS_WARN_STREAM("Couldn't find a parking space with the given id "
                            << parking_space_id << " from the jury command.");
            return;
          } 

          createParkingRegion(parking_space_id);
        } 
      }
    }
  }

  if (!foundParking && m_parking_approached) {
    // Reset flag so we can send the command again
    m_parking_approached = false;
  }

  // Check for close zebra sign:
  bool foundZebra = false;
  double zebraDist = 99999.9;
  for (auto &sign: m_tracked_objects.trafficSigns) {
    if (sign.getType() != TrafficSignType::PEDESTRIAN_CROSSING)
    {
      continue;
    }
    
    foundZebra = true;
    auto dist = sign.calcDistTo(m_ego_pose);
    zebraDist = std::min(dist, zebraDist);

    if (!m_zebra_approached && dist <= 0.75) {
      if (personNearby) {
        ROS_INFO_STREAM("PUBLISHED APPROACHING ZEBRA_CROSSING");
        ros_oadrive::Event msg;
        msg.type = "APPROACHING_ZEBRA";
        m_event_publisher.publish(msg);
      }
      else {
        ros_oadrive::Event msg;
        msg.type = "SKIPPING_ZEBRA";
        m_event_publisher.publish(msg);
      }
      m_zebra_approached = true;

      break;
    }
  }

  if (m_zebra_approached && (zebraDist > 5.0 || !foundZebra)) {
    // ROS_INFO_STREAM("PUBLISHED RESETTING ZEBRA_CROSSING");
    // ros_oadrive::Event msg;
    // msg.type = "RESETTING_ZEBRA";
    // m_event_publisher.publish(msg);
    ROS_INFO_STREAM("RESET ZEBRA_CROSSING");

    // Reset flag so we can send the command again
    m_zebra_approached = false;
  }

  // Check for children:
  bool foundChild = false;
  for (auto &person : m_tracked_objects.persons) {
    if (person.height < CHILD_HEIGHT_THRESHOLD) {
      float dist = person.object.calcDistTo(m_ego_pose);

      if (dist < 3.0) {
        foundChild = true;
      }
    }
  }

  if (!m_child_detected && foundChild) {
    // Publish true
    ROS_INFO_STREAM("Child detected and reported!");

    ros_oadrive::Event msg;
    msg.type = "ENTERED_EVENT_REGION";
    msg.event_region.type = "CHILD_REGION";
    m_event_publisher.publish(msg);

    m_child_detected = true;
  } else if (m_child_detected && !foundChild) {
    // publish gone
    ROS_INFO_STREAM("Child detected and reported!");

    ros_oadrive::Event msg;
    msg.type = "LEFT_EVENT_REGION";
    msg.event_region.type = "CHILD_REGION";
    m_event_publisher.publish(msg);

    m_child_detected = false;
  }
}

void EventTriggerNode::handleTrackedObjectEvents(const oadrive::obstacle::TrackedObject object) {
  uint64_t id = object.filterID;
  double current_distance = object.calcDistTo(m_ego_pose);

  //ROS_INFO_STREAM("Current distance to object [UID = " << id << "]: " << current_distance);

  if (m_processed_objects.find(id) == m_processed_objects.end()) {
    bool is_close = isEgoCloseToObject(object);
    bool is_in_margin = isEgoInMarginOfObject(object);
    auto dist_iter = m_object_distance.find(id);

    if (dist_iter != m_object_distance.end()) {
      double last_distance = dist_iter->second;
      bool last_time_in_margin = last_distance <= OBJECT_CLOSE_MARGIN;
      bool was_close = m_object_was_close[id];

      if (was_close && last_time_in_margin && !is_in_margin) {
        ROS_INFO_STREAM("LEFT object [UID = " << id << "]");

        m_event_publisher.publish(buildEventMessage(object, false));
        m_processed_objects.insert(id);
      } else if (!was_close && is_close) {
        ROS_INFO_STREAM("APPROACHED object [UID = " << id << "]");

        m_event_publisher.publish(buildEventMessage(object, true));
        m_object_was_close[id] = true;
      }
    } else {
      // We encounter this object for the first time.
      if (is_close) {
        ROS_INFO_STREAM("APPROACHED object [UID = " << id << "]");

        m_event_publisher.publish(buildEventMessage(object, true));
      }
      m_object_was_close[id] = is_close;
    }
    m_object_distance[id] = current_distance;
  }
}

Event EventTriggerNode::buildEventMessage(const oadrive::obstacle::TrackedObject &tracked_object,
                                          bool approached_object) {
  Event event_msg;
  event_msg.header.stamp = ros::Time::now();
  event_msg.type = approached_object ? "OBJECT_CLOSE" : "OBJECT_DISTANT";
  event_msg.tracked_object = TrackedObjectConverter::toMessage(tracked_object);
  return event_msg;
}

Event EventTriggerNode::buildEventMessage(const oadrive::world::EventRegion &event_region,
                                          bool entered_region) {
  Event event_msg;
  event_msg.header.stamp = ros::Time::now();
  event_msg.type = entered_region ? "ENTERED_EVENT_REGION" : "LEFT_EVENT_REGION";
  event_msg.event_region = EventRegionConverter::toMessage(event_region);
  return event_msg;
}

void EventTriggerNode::advertise() {
  m_event_publisher = m_node.advertise<ros_oadrive::Event>("/aadc/planning/event", 1000);
  m_event_region_publisher =
      m_node.advertise<ros_oadrive::EventRegion>("/aadc/planning/event_region", 100);
}

void EventTriggerNode::subscribe() {
  m_event_subscriber =
      m_node.subscribe("/aadc/planning/event", 1000, &EventTriggerNode::eventCallback, this);
  m_event_region_subscriber = m_node.subscribe("/aadc/planning/event_region", 1000,
                                               &EventTriggerNode::eventRegionCallback, this);
  m_tracked_objects_subscriber = m_node.subscribe("/aadc/object_tracking/tracked_objects", 1000,
                                                  &EventTriggerNode::trackedObjectsCallback, this);
  m_ego_pose_subscriber =
      m_node.subscribe("/aadc/marker_position", 10, &EventTriggerNode::poseCallback, this);
  m_current_maneuver_subscriber = m_node.subscribe(
      "/aadc/jury/current_maneuver", 10, &EventTriggerNode::currentManeuverCallback, this);
  m_state_subscriber = m_node.subscribe("/aadc/statemachine/state", 10, &EventTriggerNode::stateCallback, this);
}

bool EventTriggerNode::isEgoInEventRegion(const oadrive::world::EventRegion &event_region) {
  if (!m_pose_received) {
    return false;
  }

  // TODO(kolja): Change ego pose to ExtendedPose2d?
  return event_region.isPointInside(oadrive::core::ExtendedPose2d(m_ego_pose));
}

bool EventTriggerNode::shouldBeDiscarded(const oadrive::world::EventRegion &event_region) {
  const oadrive::core::Position2d ego_position = m_ego_pose.translation();
  double dist_to_center = event_region.calcDistTo(ego_position);

  // TODO(kolja): Only discard if event region let to trigger?
  bool should_discard = dist_to_center >= DISCARD_MIN_DISTANCE;
  if (should_discard) {
    ROS_INFO_STREAM("DISCARDED event region [UID = "
                    << event_region.getId() << "] with distance " << dist_to_center << "m > "
                    << DISCARD_MIN_DISTANCE << "m = MIN_DISTANCE.");
  }

  return should_discard;
}

bool EventTriggerNode::isEgoInMarginOfObject(const oadrive::obstacle::TrackedObject &object) {
  if (!m_pose_received) {
    return false;
  }

  return object.calcDistTo(m_ego_pose) <= OBJECT_CLOSE_MARGIN;
}

bool EventTriggerNode::isEgoCloseToObject(const oadrive::obstacle::TrackedObject &object) {
  if (!m_pose_received) {
    return false;
  }

  return object.calcDistTo(m_ego_pose) <= OBJECT_CLOSE_MAX_DISTANCE;
}

double EventTriggerNode::degreeToRadian(double degree) { return (degree/180.f) * M_PI; }

void EventTriggerNode::reset() {
  m_pose_received = false;
  m_event_regions.clear();
  m_object_distance.clear();
  m_object_was_close.clear();
  m_processed_objects.clear();

  m_tracked_objects.cars.clear();
  m_tracked_objects.persons.clear();
  m_tracked_objects.obstacles.clear();
  m_tracked_objects.trafficSigns.clear();

  m_parking_approached = false;
  m_zebra_approached = false;
  m_child_detected = false;

  ROS_INFO_STREAM("Reset successful.");
}

}  // namespace ros_oadrive

int main(int argc, char **argv) {
  ros::init(argc, argv, "EventTriggerNode");
  ros::NodeHandle nh("~");

  std::string config_folder;
  std::string parking_spaces_config_path;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  std::cout << "Config folder: " << config_folder << std::endl;
  oadrive::util::Config::setConfigPath(config_folder, car_name);

  ros_oadrive::EventTriggerNode event_trigger(nh);

  // Wait for new subscription hits.
  ros::spin();

  return 0;
}
