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
 * \author  Nico Kuhn <kuhn@fzi.de>
 * \date    2017
 * 
 * \author  Fabian Dürr
 * \date    2017
 * 
 * \author  Kai Braun <kbraun@fzi.de>
 * \date    2017
 *
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
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


#include "ros_oadrive/PlanningNode.h"

#include "ros_oadrive/EventRegionConverter.h"
#include "ros_oadrive/MultiTrajectoryConverter.h"
#include "ros_oadrive/Pose2dConverter.h"
#include "ros_oadrive/StreetPatchConverter.h"
#include "ros_oadrive/TrackedObjectsConverter.h"


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_world/Environment.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <oadrive_lanedetection/FeatureDetection/StreetTypes.h>
#include <ros_oadrive/MultiTrajectory.h>
#include <ros_oadrive/PlanningNode.h>
#include <ros_oadrive/StreetPatch.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <vector>

using namespace oadrive::world;
using namespace oadrive::util;

using oadrive::lanedetection::OadrivePose;

#define MEASURE_TIME


#define START_TIMER(t1) auto t1 = std::chrono::system_clock::now();
#define END_TIMER(t1, name) std::cout << "[TIME] " << name << ": " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t1).count() << "ms" << std::endl;

namespace ros_oadrive
{

PlanningNode::PlanningNode(ros::NodeHandle node_handle,
                           oadrive::util::CoordinateConverter* converter)
        : m_node(node_handle), mConverter(converter)
        , m_map(Config::getString("Map", "Path", ""))
        , m_last_us_occupied(std::chrono::system_clock::now())
{
  advertise();
  subscribe();

  mBypassSpeedTimer = m_node.createTimer(ros::Duration(3), &PlanningNode::bypassTimerCallback, this, true);
  mBypassSpeedTimer.stop();

}

void PlanningNode::poseCallback(const ros_oadrive::MarkerPosition::ConstPtr &pose_msg)
{
  const oadrive::core::Pose2d pose = Pose2dConverter::fromMessage(pose_msg->pose);
  oadrive::core::ExtendedPose2d extended_pose(pose);

  // Use our environment object to internally build data structures needed by
  // the TrajectoryFactory to generate trajectories.
  Environment::getInstance()->updateCarPose(extended_pose);
  publishDrivingCommand(generateLongitudinalDrivingCommand());

  // Begin: Check map to slow down when approaching intersections
  try{
    geometry_msgs::PoseStamped localPose;
    localPose.header.frame_id = "local";
    localPose.header.stamp = pose_msg->header.stamp;
    localPose.pose.position.x = pose_msg->pose.x;
    localPose.pose.position.y = pose_msg->pose.y;
    localPose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_msg->pose.theta);

    geometry_msgs::PoseStamped worldPose;
    m_tf_listener.transformPose("world", ros::Time(0), localPose, "local", worldPose);

    tf::Quaternion q(
      worldPose.pose.orientation.x,
      worldPose.pose.orientation.y,
      worldPose.pose.orientation.z,
      worldPose.pose.orientation.w
    );
    
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    auto upFrontIntersections = m_map.queryIntersections(OadrivePose(worldPose.pose.position.x, worldPose.pose.position.y, yaw), 2.5);
    auto behindIntersections = m_map.queryIntersections(OadrivePose(worldPose.pose.position.x, worldPose.pose.position.y, yaw + M_PI), 1.0);

    if (!m_slow_parking_driving && !m_slow_intersection_driving && upFrontIntersections.size() > 0) {
      ROS_INFO("Reduced speed at intersection");
      enableSlowDriving();
      m_slow_intersection_driving = true;
    } else if (!m_slow_parking_driving && m_slow_intersection_driving && upFrontIntersections.size() == 0 && behindIntersections.size() == 0) {
      ROS_INFO("Increased speed again");
      disableSlowDriving();
      m_slow_intersection_driving = false;
    }
  } catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  // End check map

}

void PlanningNode::patchCallback(const StreetPatchList::ConstPtr &patches_msg)
{
  if (m_reset_active) {
    auto timeSinceReset = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_last_reset).count();

    if (timeSinceReset > 1.0) {
      m_reset_active = false;
    } else {
      // Do not accept patches if we just resetted 
      return;
    }
  }


  MultiTrajectory msg;

  for (auto &patch : patches_msg->streetpatch)
  {
    Environment::getInstance()->addPatch(StreetPatchConverter::fromMessage(patch));
  }
  Environment::getInstance()->deleteOldPatches();

  publishJunctionEventRegions();
  publishStreetVisualization(*Environment::getInstance()->getStreet());
  // The environment instance has updated itself in the addPatch calls. We can
  // now request a new trajectory that will use the new data in the environment
  // instance.

  // START_TIMER(genTraj)
  if (Environment::getInstance()->generateNextTrajectory(m_current_state))
  {
    // std::cout << "Num Patches: " << Environment::getInstance()->getStreet()->size() << std::endl;
    // (kolja) As far as I can see, the generated MultiTrajectory will always
    // just have one single element. We should refactor in this case.
    oadrive::core::ExtendedPose2d carPose = Environment::getInstance()->getCarPose();

    msg = MultiTrajectoryConverter::toMessage(Environment::getInstance()->getMultiTrajectory());

    float yaw = carPose.getYaw();

    for (auto &traj : msg.trajectories) {
      for (int i = 0; i < traj.trajectory.size(); i++)
      {
        float localX = traj.trajectory[i].pose.x - carPose.getX();
        float localY = traj.trajectory[i].pose.y - carPose.getY();
        traj.trajectory[i].pose.x = cos(yaw) * localX + sin(yaw) * localY;
        traj.trajectory[i].pose.y = -sin(yaw) * localX + cos(yaw) * localY;
      }
    }

    // START_TIMER(pubTraj)
    populateMessageHeader(msg);

    m_trajectory_publisher.publish(msg);
    // END_TIMER(pubTraj, "PubTraj")
  }

  // END_TIMER(genTraj, "GenerateTraj")

  // START_TIMER(showDbg)
  // showDebugImage();
  // END_TIMER(showDbg, "showDbg")
}

void PlanningNode::reset() {
  m_last_reset = std::chrono::system_clock::now();
  m_reset_active = true;

  m_slow_bypass_driving = false;
  m_slow_intersection_driving = false;
  m_slow_parking_driving = false;
  m_cross_section_halt = false;

  Environment::reset();
  m_entered_junction_regions.clear();
  Environment::getInstance()->setLane(LANE_RIGHT);
}

void PlanningNode::bypassTimerCallback(const ros::TimerEvent& event) {
  m_slow_bypass_driving = false;
      
  if (!m_slow_intersection_driving && !m_slow_parking_driving) {
    disableSlowDriving();
  }
}


void PlanningNode::stateCallback(const StateChange::ConstPtr &msg)
{
  auto prevState = static_cast<StateMachine::State>(msg->prevState);
  auto newState = static_cast<StateMachine::State>(msg->newState);

  if (prevState == StateMachine::State::ACTIVE || prevState == StateMachine::State::INACTIVE) {
    reset();
  }

  m_current_state = newState;

  // handle entered states
  switch(newState) {
    // entered intersection state
    case StateMachine::TURNING_LEFT:
    case StateMachine::TURNING_RIGHT:
    case StateMachine::DRIVING_STRAIGHT_AT_INTERSECTION:
    {
      // This is currently handled by the event region for the CROSS_SECTION_HALT
      // Environment::getInstance()->processEvent(oadrive::world::Event::APPROACH_INTERSECTION);
      // publishDrivingCommand(generateLongitudinalDrivingCommand());
      break;
    }
    case StateMachine::PRE_BYPASSING:
    {
      enableSlowDriving();
      m_slow_bypass_driving = true;
      break;
    }
    case StateMachine::FORMING_RESCUE_LANE:
      ROS_INFO_STREAM("Switching to rescue lane");
      Environment::getInstance()->setLane(RESCUE_LANE);
      break;
    case StateMachine::WAITING_IN_RESCUE_LANE:
      ROS_INFO_STREAM("Waiting to rescue lane");
      publishDrivingCommand(generateLongitudinalDrivingCommand());
      break;
    case StateMachine::UNFORMING_RESCUE_LANE:
      ROS_INFO_STREAM("Switching to right lane");
      Environment::getInstance()->setLane(LANE_RIGHT);
      break;
    case StateMachine::SWITCHING_FOR_BYPASS:
      Environment::getInstance()->processEvent(oadrive::world::Event::TRAJECTORY_FINISHED);
      // Environment::getInstance()->setLane(SWITCH_TARGET_LANE);
      Environment::getInstance()->setLane(LANE_LEFT);
      ROS_INFO_STREAM("Switching to left lane");
      break;
    case StateMachine::CURRENTLY_BYPASSING:
      ROS_INFO_STREAM("Currently Bypassing");
      Environment::getInstance()->setLane(LANE_LEFT);
      break;
    case StateMachine::SWITCHING_BACK_FOR_BYPASS:
      ROS_INFO_STREAM("Switching to right lane");
      Environment::getInstance()->setLane(LANE_RIGHT);
      // Environment::getInstance()->setLane(SWITCH_BACK_TARGET_LANE);
      break;
    case StateMachine::SLOW_DRIVING:
    {
      ROS_INFO("Reduced speed for child");
      enableSlowDriving();
      break;
    }
    case StateMachine::DRIVING_RAMP:
    {
      ROS_INFO("Reduced speed for ramp");
      enableSlowDriving();
      break;
    }
    case StateMachine::PRE_MERGING:
    {
      ROS_INFO("Reduced speed for merging");
      enableSlowDriving();
      break;
    }
    case StateMachine::CURRENTLY_MERGING:
    {
      // Set time for start of merging
      m_merging_start = std::chrono::system_clock::now();
      m_merging_done = false;
      break;
    }
    default:
      break;
  }

  // handle left states
  switch(prevState) {
    // left intersection state
    case StateMachine::TURNING_LEFT:
    case StateMachine::TURNING_RIGHT:
    case StateMachine::DRIVING_STRAIGHT_AT_INTERSECTION:
    {
      // Environment::getInstance()->processEvent(oadrive::world::Event::TRAJECTORY_FINISHED);
      Environment::getInstance()->processEvent(oadrive::world::Event::FINISH_INTERSECTION);
      m_cross_section_halt = false; 
      ROS_INFO_STREAM("LEFT INTERSECTION, m_cross_section_halt resetted!");
      break;
    }
    case StateMachine::PULLING_OUT_LEFT:
    case StateMachine::PULLING_OUT_RIGHT:
    case StateMachine::PARKING_LEFT:
    case StateMachine::PARKING_RIGHT:
    {
      // speed up the car again:
      disableSlowDriving();
      m_slow_parking_driving = false;
      Environment::getInstance()->processEvent(oadrive::world::Event::TRAJECTORY_FINISHED);
      Environment::getInstance()->processEvent(oadrive::world::Event::RESET_ROAD);
      break;
    }
    case StateMachine::RESCUE_LANE:
    {
      Environment::getInstance()->setLane(LANE_RIGHT);
      break;
    }
    case StateMachine::BYPASS:
    {
      mBypassSpeedTimer.start();
      Environment::getInstance()->setLane(LANE_RIGHT);
      break;
    }
    case StateMachine::SLOW_DRIVING:
    {
      ROS_INFO("Increased speed after child again");
      disableSlowDriving();
      break;
    }
    case StateMachine::MERGING:
    {
      ROS_INFO("Increase speed after merging");
      disableSlowDriving();
      break;
    }
    case StateMachine::DRIVING_RAMP:
    {
      ROS_INFO("RAMP FINISHED");
      Environment::getInstance()->processEvent(oadrive::world::Event::TRAJECTORY_FINISHED);
      Environment::getInstance()->processEvent(oadrive::world::Event::RESET_ROAD);
      break;
    }
    default:
      break;
  }
}

void PlanningNode::eventCallback(const ros_oadrive::Event::ConstPtr& msg) {
  std::string type = msg->type;

  if (type == "ENTERED_EVENT_REGION") {
    
    ROS_INFO_STREAM("EVENT REGION");
    // only load takeoff point from parking space region
    if (msg->event_region.type == "PARKING_SPACE_REGION") {
      ROS_INFO_STREAM("PARKING REGION");
      oadrive::core::Pose2d pose = Pose2dConverter::fromMessage(msg->event_region.parking_take_off);
      Environment::getInstance()->setParkingTakeoffPoint(pose);
    } else if (msg->event_region.type == "CROSS_SECTION_REGION") {
      auto event_region = EventRegionConverter::fromMessage(msg->event_region);
      ROS_INFO_STREAM("ENTERED CROSS SECTION REGION " << event_region.getId());
      m_entered_junction_regions.insert(event_region.getId());
    } else if (msg->event_region.type == "CROSS_SECTION_HALT") {
      auto event_region = EventRegionConverter::fromMessage(msg->event_region);
      ROS_INFO_STREAM("CROSS_SECTION_HALT REACHED! " << event_region.getId() << " was halting: " << m_cross_section_halt);
      m_cross_section_halt = true;
      m_entered_junction_regions.insert(event_region.getId());

      Environment::getInstance()->processEvent(oadrive::world::Event::APPROACH_INTERSECTION);
      publishDrivingCommand(generateLongitudinalDrivingCommand());
    }
  } else if (type == "LEFT_EVENT_REGION") {
      ROS_INFO_STREAM("LEFT_EVENT_REGION REGION");
      // only load takeoff point from parking space region
      if (msg->event_region.type == "CROSS_SECTION_REGION") {
        auto event_region = EventRegionConverter::fromMessage(msg->event_region);
        ROS_INFO_STREAM("LEFT CROSS SECTION REGION " << event_region.getId());
      } else if (msg->event_region.type == "CROSS_SECTION_HALT") {
        auto event_region = EventRegionConverter::fromMessage(msg->event_region);
        ROS_INFO_STREAM("CROSS_SECTION_HALT LEFT! " << event_region.getId() << " was halting: " << m_cross_section_halt);
        m_cross_section_halt = false; 
      }
    } else if (type == "APPROACHING_PARKING_SPACES") {
    ROS_INFO_STREAM("APPROACHING PARKING REGION");

    enableSlowParkDriving();
    m_slow_parking_driving = true;
    m_slow_intersection_driving = false;
  }
}

void PlanningNode::mapCallback(const std_msgs::String::ConstPtr &msg) {
  m_map.loadFromString(msg->data);
}

void PlanningNode::patchOccupancyCallback(const ros_oadrive::StreetPatchOccupancy::ConstPtr &msg) {
  // skip car2x msgs
  if (msg->external_observations) {
    return;
  }
  // First we store the recommended lane, then we also set the lane on other patches around
  // This allows us to reverse the process if the patch clears again
  PatchPtr prev = PatchPtr();
  // First find the relevant patch - we set the occupancy for this to make reversing easier
  for (auto &patch : *Environment::getInstance()->getStreet()) {
    if (patch->getPatchID() == msg->patch_id) {
      break;
    } else if (patch->getPatchID() < msg->patch_id) {
      prev = patch;
    }
  }
  
  if (prev) {
    LaneType target = LANE_RIGHT;

    // Determine the recommended lane
    if (m_current_state != StateMachine::State::WAITING_AT_ZEBRA_CROSSING) {
      if (msg->right && msg->switch_possible && !msg->strong_switch) {
        if (prev->getRecommendedLane() != LANE_CENTER) {
          ROS_INFO_STREAM("OCCUPANCY: " << prev->getPatchID() << " occupied! CENTER set");
        }
        target = LANE_CENTER;
      // } else if (msg->right && msg->switch_possible && msg->strong_switch) {
      //   if (prev->getRecommendedLane() != LANE_LEFT) {
      //     ROS_INFO_STREAM("OCCUPANCY: " << prev->getPatchID() << " occupied! LEFT set");
      //   }
      //   target = LANE_LEFT;
      // Removed as is causes trouble
      // } else if ((!msg->right || !msg->switch_possible)) {
      //   if (prev->getRecommendedLane() != LANE_RIGHT) {
      //     ROS_INFO_STREAM("OCCUPANCY: " << prev->getPatchID() << " cleared! RIGHT set");
      //   }
      //   target = LANE_RIGHT;
      } else {
        return;
      }
    } else {
      return;
      // target = LANE_RIGHT;
    }

    int targetLevel = getLaneLevel(target);

    if ((target == LANE_RIGHT || target == LANE_CENTER) && prev->getRecommendedLane() != target) {
      prev->setRecommendedLane(target);

      auto currentPatch = prev;
      for (int i = 0; i < 4; i++) {
        // Do not change left lane or switch lanes, to not destroy the bypass
        if (currentPatch->getLane() == LANE_RIGHT || currentPatch->getLane() == LANE_CENTER) {
          currentPatch->setLane(target);
        }
        currentPatch = currentPatch->getSuccessor();

        // If the patch to be changed has an higher recommended lane, we wont change any patches starting from this one
        // E.g. If we are setting the patches to center and the patch to be changed is left, we will stop here
        // If it is the other way around, we will switch the center patch to left
        if (!currentPatch || getLaneLevel(currentPatch->getRecommendedLane()) > targetLevel) {
          break;
        }
      }
    }
  }
}

void PlanningNode::trackedObjectCallback(const TrackedObjects::ConstPtr &trackedObjects)
{
  oadrive::obstacle::TrackedObjects oadrivetrackedObjects = TrackedObjectsConverter::fromMessage(
          *trackedObjects);

  Environment::getInstance()->resetTrafficCars();
  for (int i = 0; i < oadrivetrackedObjects.cars.size(); i++)
  {
    Environment::getInstance()->addTrafficCar(oadrivetrackedObjects.cars[i]);
  }

  for (int i = 0; i < oadrivetrackedObjects.trafficSigns.size(); i++)
  {
    Environment::getInstance()->addTrafficSign(oadrivetrackedObjects.trafficSigns[i]);
  }

  publishDrivingCommand(generateLongitudinalDrivingCommand());
}

void PlanningNode::usCallback(const ros_oadrive::Ultrasonic::ConstPtr& msg)
{
  m_last_us = *msg;
}

oadrive::world::DrivingCommand PlanningNode::generateLongitudinalDrivingCommand() {
  oadrive::world::DrivingCommand nextCommand = oadrive::world::DrivingCommand::DRIVE;

  switch (m_current_state) {
    case StateMachine::TURNING_RIGHT:
    case StateMachine::TURNING_LEFT:
    case StateMachine::DRIVING_STRAIGHT_AT_INTERSECTION:
      if (m_cross_section_halt) {
        nextCommand = Environment::getInstance()->checkForStop() ? oadrive::world::DrivingCommand::STOP : oadrive::world::DrivingCommand::DRIVE;
      }
      break;
    case StateMachine::WAITING_IN_RESCUE_LANE:
      nextCommand = oadrive::world::DrivingCommand::STOP;
      break;
    case StateMachine::PRE_MERGING:
    {
      // Already sample the ultrasonic to check if there is something slow on the main lane
      mergingCheckForStop();
      break;
    }
    case StateMachine::CURRENTLY_MERGING:
    {
      // It is important to always call the checkforstop method to collect us informations while waiting!
      bool stop = mergingCheckForStop();

      if (stop) {
        nextCommand = oadrive::world::DrivingCommand::STOP;
      }

      auto mergingTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_merging_start).count();
      ROS_INFO_STREAM("Waiting for merge since " << mergingTime << " seconds.. wait for stop says: " << stop);
      if (mergingTime < 1.5) {
        nextCommand = oadrive::world::DrivingCommand::STOP;
      } else if (m_merging_done) {
        // If we already waited we should not stop anymore, because there might be things on the other lane
        nextCommand = oadrive::world::DrivingCommand::DRIVE;
      } else if (nextCommand == oadrive::world::DrivingCommand::DRIVE) {
        // Set that merging is done for further checks
        m_merging_done = true;
      }
      break;
    }
    default:
      nextCommand = oadrive::world::DrivingCommand::DRIVE;
      break;
  }

  if (m_last_command != nextCommand)
  {
    if (nextCommand == oadrive::world::DrivingCommand::STOP) {
      m_last_stop = std::chrono::system_clock::now();
    }

    m_last_command = nextCommand;
    return nextCommand;
  }
  else
  {
    // We should put an timeout here to prevent too long waiting!
    if (m_last_command == oadrive::world::DrivingCommand::STOP) {
      auto timePassed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_last_stop).count();

      // If we are stopping for more than 20 seconds, a timeout should kick in and keep us driving again!
      // Hopefully this will save us!
      if (timePassed > 10) {
        ROS_INFO_STREAM("We waited for more than 10 seconds, so we will continue driving now..");
        return oadrive::world::DrivingCommand::DRIVE;
      }
    }
    
    return DrivingCommand::NONE;
  }
}

bool PlanningNode::mergingCheckForStop() {
  // we stop if there is a car to close, then we wait at least 3 seconds after it passed
  auto timeSinceLastOccupied = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_last_us_occupied).count();
  if (m_last_us.sideLeft > 0 && m_last_us.sideLeft < 40) {
    m_last_us_occupied = std::chrono::system_clock::now();
    return true;
  } else if (m_last_us.rearLeft > 0 && m_last_us.rearLeft < 140) {
    m_last_us_occupied = std::chrono::system_clock::now();
    return true;
  } else if (timeSinceLastOccupied < 3) {
    return true;
  }

  return false;
}

void PlanningNode::publishDrivingCommand(oadrive::world::DrivingCommand drivingCommand)
{
  std::string drivingCommandAsString = "";
  switch (drivingCommand)
  {
    case oadrive::world::DrivingCommand::STOP:
      drivingCommandAsString = "Stop";
      break;
    case oadrive::world::DrivingCommand::DRIVE:
      drivingCommandAsString = "Start";
      break;
    case oadrive::world::DrivingCommand::NONE:
      return;
  }

  std_msgs::String msg;
  msg.data = drivingCommandAsString;
  m_longitudinalCommand_publisher.publish(msg);
}

double PlanningNode::distanceToPatch(PatchPtr patch)
{
  oadrive::core::ExtendedPose2d ego_position = Environment::getInstance()->getCarPose();
  double dx = ego_position.getX() - patch->getX();
  double dy = ego_position.getY() - patch->getY();

  return std::sqrt(dx * dx + dy * dy);
}

void PlanningNode::populateMessageHeader(MultiTrajectory &msg)
{
  msg.header.stamp = ros::Time::now();
}

void PlanningNode::publishJunctionEventRegions()
{
  const PatchPtrList* street = Environment::getInstance()->getStreet();
  PatchPtrList::const_iterator it = street->begin();

  while (it != street->end())
  {
    // Check that we did not already enter the crossing before!
    if ((*it)->getPatchType() == CROSS_SECTION &&
        m_entered_junction_regions.find((*it)->getPatchID()) ==
        m_entered_junction_regions.end())
    {
      double yaw = (*it)->getYaw();
      oadrive::core::Pose2d pose;
      oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromOrientationRPY(pose, yaw);
      pose.translation()[0] =
              (*it)->getX() - std::cos(yaw) * JUNCTION_EXPANSION_LENGTH_IN_METERS / 2;
      pose.translation()[1] =
              (*it)->getY() - std::sin(yaw) * JUNCTION_EXPANSION_LENGTH_IN_METERS / 2;

      double width = (*it)->getWidth();
      double length = (*it)->getLength() + JUNCTION_EXPANSION_LENGTH_IN_METERS;

      oadrive::core::Pose2d takeoffDummy;
      oadrive::world::EventRegion event_region(CROSS_SECTION_REGION, pose, takeoffDummy, width, length, true);
      event_region.setId((*it)->getPatchID());

      m_event_region_publisher.publish(EventRegionConverter::toMessage(event_region));


      // Stop region
      oadrive::core::Pose2d poseHalt;
      oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromOrientationRPY(poseHalt, yaw);
      poseHalt.translation()[0] =
              (*it)->getX() - std::cos(yaw) * JUNCTION_HALT_EXPANSION_LENGTH_IN_METERS / 2;
      poseHalt.translation()[1] =
              (*it)->getY() - std::sin(yaw) * JUNCTION_HALT_EXPANSION_LENGTH_IN_METERS / 2;

      width = (*it)->getWidth();
      length = (*it)->getLength() + JUNCTION_HALT_EXPANSION_LENGTH_IN_METERS;

      oadrive::world::EventRegion event_region_halt(CROSS_SECTION_HALT, poseHalt, takeoffDummy, width, length, true);
      event_region_halt.setId((*it)->getPatchID() + 1);

      m_event_region_publisher.publish(EventRegionConverter::toMessage(event_region_halt));
    }

    ++it;
  }
}

void PlanningNode::publishStreetVisualization(PatchPtrList street)
{
  int id = 0;
  for (boost::shared_ptr<Patch> p : street)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "local";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.lifetime = ros::Duration(3.0);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = p->getX();
    marker.pose.position.y = p->getY();
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.f;
    marker.pose.orientation.y = 0.f;
    marker.pose.orientation.z = sin(p->getYaw() / 2.f);
    marker.pose.orientation.w = cos(p->getYaw() / 2.f);

    if (p->getPatchType() == CROSS_SECTION) {
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 0.01;

      marker.color.a = 0.5;  // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.scale.x = 0.25;
      marker.scale.y = 1.0;
      marker.scale.z = 0.01;

      marker.color.a = 0.5;  // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.7;
      marker.color.b = 0.0;
    }
    
    m_rviz_street_publisher.publish(marker);
    id++;
  }
}

void PlanningNode::advertise()
{
  m_trajectory_publisher = m_node.advertise<MultiTrajectory>("/aadc/planning/trajectory", 1);
  m_event_region_publisher = m_node.advertise<EventRegion>("/aadc/planning/event_region", 1);
  m_longitudinalCommand_publisher = m_node.advertise<std_msgs::String>("/aadc/jury/status", 1);
  m_event_publisher = m_node.advertise<Event>("/aadc/planning/event", 1);
  m_speed_publisher = m_node.advertise<std_msgs::Float64>("/aadc/statemachine/targetspeed", 1);

  m_rviz_street_publisher =
        m_node.advertise<visualization_msgs::Marker>("/aadc/visualization_street", 100);

}

void PlanningNode::subscribe()
{
  m_pose_subscriber = m_node.subscribe("/aadc/marker_position", 1, &PlanningNode::poseCallback, this);
  m_patch_subscriber =
          m_node.subscribe("/aadc/perception/patches", 1, &PlanningNode::patchCallback, this);
  m_trackedObject_subscriber = m_node.subscribe("/aadc/object_tracking/tracked_objects", 1, &PlanningNode::trackedObjectCallback, this);
  m_state_subscriber = m_node.subscribe("/aadc/statemachine/state", 10, &PlanningNode::stateCallback, this);

  m_us_subscriber = m_node.subscribe("/aadc/ultrasonic", 5, &PlanningNode::usCallback, this);

  m_event_subscriber = m_node.subscribe("/aadc/planning/event", 100, &PlanningNode::eventCallback, this);

  m_map_subscriber = m_node.subscribe("/aadc/alpaka_map", 1000, &PlanningNode::mapCallback, this);

  m_patch_occupancy_subscriber = m_node.subscribe("/aadc/planning/patch_occupancy", 100, &PlanningNode::patchOccupancyCallback, this);

  try {
    ROS_INFO("Waiting for transformations to be available..");
    m_tf_listener.waitForTransform("world", "local", ros::Time(0), ros::Duration(20.0));
    ROS_INFO("..Done!");
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }
}

}  // namespace ros_oadrive

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PlanningNode");
  ros::NodeHandle nh("~");

  std::string config_folder;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  // Init stuff to get this going.
  oadrive::util::Config::setConfigPath(config_folder, car_name);
  oadrive::world::Environment::init();

  std::string mBirdViewCalFile(
          (boost::filesystem::path(config_folder) / car_name / "BirdviewCal.yml").string());

  CoordinateConverter* converter = new CoordinateConverter(mBirdViewCalFile);

  ros_oadrive::PlanningNode trajectory_generator(nh, converter);

  // Wait for new subscription hits.
  ros::spin();

  return 0;
}
