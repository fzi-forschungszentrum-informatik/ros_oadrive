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
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include <ros_oadrive/StateMachineNode.h>
#include <oadrive_util/Config.h>

using namespace ros_oadrive;

StateMachineNode::StateMachineNode(ros::NodeHandle nh) : mStateMachine()
{
    mNodeHandle = nh;
    mCurrentState = StateMachine::INACTIVE;
    mCurrentManeuver = "straight";
    mTargetSpeed = oadrive::util::Config::getDouble("Driver", "NormalSpeed", 0.75);
    mEmergencyBrakeActivated = false;
    mParked = false;
    
    mJuryTimer = nh.createTimer(ros::Duration(5), &StateMachineNode::timerCallback, this, true);
    mBypassTimer = nh.createTimer(ros::Duration(5), &StateMachineNode::timerCallback, this, true);
    mEmergencyVehicleTimer = nh.createTimer(ros::Duration(5), &StateMachineNode::timerCallback, this, true);
    mParkingTimer = nh.createTimer(ros::Duration(5), &StateMachineNode::timerCallback, this, true);
    mZebraTimer = nh.createTimer(ros::Duration(5), &StateMachineNode::timerCallback, this, true);
    mWaitingTimer = nh.createTimer(ros::Duration(5), &StateMachineNode::timerCallback, this, true);

    mJuryTimer.stop();
    mEmergencyVehicleTimer.stop();
    mParkingTimer.stop();
    mZebraTimer.stop();
    mBypassTimer.stop();
    mWaitingTimer.stop();

    advertise();
    subscribe();
}

void StateMachineNode::advertise()
{
    mStatePub = mNodeHandle.advertise<ros_oadrive::StateChange>("/aadc/statemachine/state", 10);
    mJuryPub = mNodeHandle.advertise<std_msgs::String>("/aadc/jury/event", 10);
    mDrivingCommandPub = mNodeHandle.advertise<ros_oadrive::ControllerCommand>("/aadc/driver_module/command", 100);
}

void StateMachineNode::subscribe()
{
    mEventSub = mNodeHandle.subscribe("/aadc/planning/event", 10, &StateMachineNode::eventLogic, this);
    mJuryStateSub = mNodeHandle.subscribe("/aadc/jury/status", 10, &StateMachineNode::juryStateLogic, this);
    mJuryEventSub = mNodeHandle.subscribe("/aadc/jury/event", 10, &StateMachineNode::juryEventLogic, this);
    mManeuverSub = mNodeHandle.subscribe("/aadc/jury/current_maneuver", 1000, &StateMachineNode::maneuverLogic, this);
    mTargetSpeedSub = mNodeHandle.subscribe("/aadc/statemachine/targetspeed", 10, &StateMachineNode::targetSpeedLogic, this); 
    mEmergencyBrakeSub = mNodeHandle.subscribe("/aadc/statemachine/emergency_brake", 1, &StateMachineNode::emergencyBrakeLogic, this);
}

void StateMachineNode::eventLogic(const ros_oadrive::Event::ConstPtr& event) {
  std::string type = event->type;

  // reset transition is special case, because previous state has to make clear that a reset happened
  if (type == "RESET") {
    mStateMachine.triggerEvent(StateMachine::Event::RESET);
    publishTransition(StateMachine::State::ACTIVE, StateMachine::State::IDLING);
    mCurrentState = StateMachine::State::IDLING;
    
    // Reset event region id
    mLastManeuverEventRegion = -1;
    mCrossSectionCounter = 0;
    return;
  }

  // evoke corresponding transition
  if (type == "ENTERED_EVENT_REGION") {

    // entered cross section region
    if (event->event_region.type == "CROSS_SECTION_REGION") {
      mCrossSectionCounter++;
      if (mCurrentManeuver == "left") {
        mStateMachine.triggerEvent(StateMachine::Event::TURN_LEFT);
      }
      else if (mCurrentManeuver == "right") {
        mStateMachine.triggerEvent(StateMachine::Event::TURN_RIGHT);
      }
      else {
        mStateMachine.triggerEvent(StateMachine::Event::DRIVE_STRAIGHT);
      }
           
      if (mNewManeuver && mLastManeuverEventRegion != event->event_region.uid) {
        // move maneuver list forward
        std_msgs::String msg;
        msg.data = "MANEUVER_DONE_EVENT";
        mJuryPub.publish(msg);
        mNewManeuver = false;
        mLastManeuverEventRegion = event->event_region.uid;
        ROS_INFO_STREAM("ENTERED CROSS_SECTION_REGION " << mLastManeuverEventRegion);
      } else if (mLastManeuverEventRegion == event->event_region.uid) {
        ROS_WARN_STREAM("ENTERED CROSS_SECTION_REGION twice, skipped! Id: " << event->event_region.uid);
      }
    }
    else if (event->event_region.type == "PARKING_SPACE_REGION") {
      mStateMachine.triggerEvent(StateMachine::Event::START_PARKING_RIGHT);
    }
    // entered merge region
    else if (event->event_region.type == "MERGE_REGION") {
      mStateMachine.triggerEvent(StateMachine::Event::START_MERGE);
    }
    else if (event->event_region.type == "EMERGENCY_VEHICLE_REGION") {
      mStateMachine.triggerEvent(StateMachine::Event::START_FORMING_RESCUE_LANE);
      mEmergencyVehicleTimer.setPeriod(ros::Duration(1.5));
      mEmergencyVehicleTimer.start();
    }
    else if (event->event_region.type == "CHILD_REGION") {
      mStateMachine.triggerEvent(StateMachine::Event::ACTIVATE_SLOW_DRIVING);
    }
  }
  else if (type == "LEFT_EVENT_REGION") {

    // left cross section region
    if (event->event_region.type == "CROSS_SECTION_REGION") {
      mCrossSectionCounter--;

      if (mCrossSectionCounter < 0) {
        ROS_WARN_STREAM("mCrossSectionCounter was " << mCrossSectionCounter << " - RESET to zero");
        mCrossSectionCounter = 0;
      }
      ROS_INFO_STREAM("mCrossSectionCounter " << mCrossSectionCounter);
      if (mCrossSectionCounter == 0) {
        mStateMachine.triggerEvent(StateMachine::Event::LEAVE_INTERSECTION);
      }
    }
    // left merge region
    else if (event->event_region.type == "MERGE_REGION") {
      mStateMachine.triggerEvent(StateMachine::Event::FINISH_MERGE);
    }
    else if (event->event_region.type == "EMERGENCY_VEHICLE_REGION") {
      mStateMachine.triggerEvent(StateMachine::Event::START_UNFORMING_RESCUE_LANE);
      mEmergencyVehicleTimer.setPeriod(ros::Duration(1.5));
      mEmergencyVehicleTimer.start();
    }
    else if (event->event_region.type == "CHILD_REGION") {
      mStateMachine.triggerEvent(StateMachine::Event::DEACTIVATE_SLOW_DRIVING);
    }
  }
  else if (type == "TRAJECTORY_FINISHED") {
    mStateMachine.triggerEvent(StateMachine::Event::FINISH_TRAJECTORY);

    if ((mCurrentManeuver == "pull_out_left" || mCurrentManeuver == "pull_out_right") && mNewManeuver) {
        std_msgs::String msg;
        msg.data = "MANEUVER_DONE_EVENT";
        mJuryPub.publish(msg);
        mNewManeuver = false;
        mLastManeuverEventRegion = -1;
    }
    else if ((mCurrentManeuver == "cross_parking") && mNewManeuver 
              && (mCurrentState == StateMachine::PARKING_LEFT
              || mCurrentState == StateMachine::PARKING_RIGHT)) {
        mParkingTimer.setPeriod(ros::Duration(4));
        mParkingTimer.start();
        sendHaltCommand();
    }
  }
  else if (type == "MERGE_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::PERFORM_MERGE);

    if (mCurrentManeuver == "merge_left" && mNewManeuver) {
      std_msgs::String msg;
      msg.data = "MANEUVER_DONE_EVENT";
      mJuryPub.publish(msg);
      mNewManeuver = false;
      mLastManeuverEventRegion = -1;
    }
  }
  else if (type == "BYPASS_START_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::START_BYPASS);
    mBypassTimer.setPeriod(ros::Duration(2.5));
    mBypassTimer.start();
  }
  else if (type == "BYPASS_END_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::START_SWITCHING_BACK_FOR_BYPASS);
    mBypassTimer.setPeriod(ros::Duration(1.5));
    mBypassTimer.start();
  }
  else if (type == "PULL_OUT_LEFT_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::START_PULLING_OUT_LEFT);
  }
  else if (type == "PULL_OUT_RIGHT_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::START_PULLING_OUT_RIGHT);
  }
  else if (type == "PARK_LEFT_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::START_PARKING_LEFT);
  }
  else if (type == "PARK_RIGHT_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::START_PARKING_RIGHT);
  }
  else if (type == "APPROACHING_ZEBRA") {
    mStateMachine.triggerEvent(StateMachine::Event::APPROACH_ZEBRA_CROSSING);
    if (mStateMachine.queryState() != mCurrentState) {
      mZebraTimer.setPeriod(ros::Duration(8));
      mZebraTimer.start();
      sendHaltCommand();
    }
  }
  else if (type == "SKIPPING_ZEBRA") {
    mStateMachine.triggerEvent(StateMachine::Event::DRIVE_THROUGH_ZEBRA);
    if (mStateMachine.queryState() != mCurrentState) {
      mZebraTimer.setPeriod(ros::Duration(1));
      mZebraTimer.start();
      sendHaltCommand();
    }
  }
  else if (type == "LEAVING_ZEBRA") {
    mStateMachine.triggerEvent(StateMachine::Event::LEAVE_ZEBRA_CROSSING);
  }
  else if (type == "APPROACHING_RAMP_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::APPROACH_RAMP);
  }
  else if (type == "START_TELE_OP_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::START_TELE_OP);
  }
  else if (type == "END_TELE_OP_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::STOP_TELE_OP);
  }
  else if (type == "START_WAITING_EVENT") {
    mStateMachine.triggerEvent(StateMachine::Event::START_WAITING);
    mWaitingTimer.setPeriod(ros::Duration(5));
    mWaitingTimer.start();
    sendHaltCommand();
  }

  // get resulting state
  StateMachine::State newState = mStateMachine.queryState();

  // no transition was taken
  if (newState == mCurrentState) {
    ROS_INFO_STREAM("No transition taken");
    return;
  }

  // special treatment in some cases that have to deal with superstates
  if (type == "LEFT_EVENT_REGION" && event->event_region.type == "MERGE_REGION") {
    publishTransition(StateMachine::State::MERGING, newState);
  }
  else {
    publishTransition(mCurrentState, newState);
  }  

  mCurrentState = newState;

  return;
}

void StateMachineNode::maneuverLogic(const ros_oadrive::JuryManeuver::ConstPtr &msg) {
  mCurrentManeuver = msg->action;

  mNewManeuver = true;

  if (mCurrentManeuver == "pull_out_left") {
    mStateMachine.triggerEvent(StateMachine::Event::START_PULLING_OUT_LEFT);
    sendDriveCommand();
  }
  else if (mCurrentManeuver == "pull_out_right") {
    mStateMachine.triggerEvent(StateMachine::Event::START_PULLING_OUT_RIGHT);
    sendDriveCommand();
  }
  // get resulting state
  StateMachine::State newState = mStateMachine.queryState();

  // no transition was taken
  if (newState == mCurrentState) {
    ROS_INFO_STREAM("No transition taken");
    return;
  }

  publishTransition(mCurrentState, newState);

  mCurrentState = newState;

  return;
}

void StateMachineNode::juryEventLogic(const std_msgs::String::ConstPtr& msg) {
  auto event = msg->data;

  if (event == "MANEUVER_DONE_EVENT") {
    return;
  }
  else if (event == "GET_READY_EVENT") {

    mJuryReady = false;

    // Same action as reset
    mStateMachine.triggerEvent(StateMachine::Event::JURY_PREPARE);
    /*publishTransition(StateMachine::State::INACTIVE, StateMachine::State::IDLING);
    mCurrentState = StateMachine::State::IDLING;*/

    mJuryTimer.setPeriod(ros::Duration(1));
    mJuryTimer.start();

    sendHaltCommand();

    if (mCurrentState != StateMachine::State::INACTIVE) {
      publishTransition(StateMachine::State::ACTIVE, StateMachine::State::IDLING);
      mCurrentState = StateMachine::State::IDLING;
      return;
    }

  } else if (event == "START_EVENT") {
    mJuryStarted = true;
    mStateMachine.triggerEvent(StateMachine::Event::JURY_START);
    sendDriveCommand();
  } else if (event == "STOP_EVENT") {
    mJuryStarted = false;
    mStateMachine.triggerEvent(StateMachine::Event::JURY_STOP);
    sendHaltCommand();
  }

  StateMachine::State newState = mStateMachine.queryState();

  if (newState == mCurrentState) {
    ROS_INFO_STREAM("ERROR: No transition taken. THIS SHOULD NEVER HAPPEN BECAUSE IT MEANS THAT A JURY COMMAND IS BEING IGNORED");
    return;
  }

  publishTransition(mCurrentState, newState);

  mCurrentState = newState;

  return;
}

void StateMachineNode::juryStateLogic(const std_msgs::String::ConstPtr& msg) {
  std::string state = msg->data.c_str();

  if (state == "Start") {
    mStateMachine.triggerEvent(StateMachine::Event::START);

    //Send isReady Signal
    std_msgs::String msg;
    msg.data = "RUNNING";

    sendDriveCommand();
  }
  if (msg->data.c_str() == std::string("Stop")) {
    mStateMachine.triggerEvent(StateMachine::Event::STOP);

    sendHaltCommand();
  }

  // get resulting state
  StateMachine::State newState = mStateMachine.queryState();

  // no transition was taken
  if (newState == mCurrentState) {
    ROS_INFO_STREAM("No transition taken");
    return;
  }

  publishTransition(mCurrentState, newState);

  mCurrentState = newState;

  return;
}

void StateMachineNode::targetSpeedLogic(const std_msgs::Float64::ConstPtr& msg) {
  mTargetSpeed = msg->data;

  if (mRunning) {
    sendDriveCommand();
  }
}

void StateMachineNode::emergencyBrakeLogic(const ros_oadrive::EmergencyBrake::ConstPtr& msg) {
  bool em_state = msg->emergency_brake;

  // em_state = em_state && mCurrentState != StateMachine::State::PRE_BYPASSING 
  //                     && mCurrentState != StateMachine::State::SWITCHING_FOR_BYPASS
  //                     && mCurrentState != StateMachine::State::DRIVING_RAMP
  //                     && mCurrentState != StateMachine::State::PARKING_LEFT
  //                     && mCurrentState != StateMachine::State::PARKING_RIGHT;

  em_state = em_state && (
    mCurrentState == StateMachine::State::NORMAL_DRIVING ||
    mCurrentState == StateMachine::State::SLOW_DRIVING ||
    mCurrentState == StateMachine::State::TURNING_LEFT ||
    mCurrentState == StateMachine::State::TURNING_RIGHT ||
    mCurrentState == StateMachine::State::DRIVING_STRAIGHT_AT_INTERSECTION ||
    mCurrentState == StateMachine::State::CURRENTLY_MERGING
  );

  if (em_state != mEmergencyBrakeActivated) {
    // stop the car
    if (em_state) {
      sendEmergencyHaltCommand();
    }
    // continue driving
    else {
      sendDriveCommand();
    }

    mEmergencyBrakeActivated = em_state;
  }
}

void StateMachineNode::sendDriveCommand() {
  if (!mJuryStarted || mCurrentState == StateMachine::State::PARKING/* || 
                       mCurrentState == StateMachine::State::WAITING_AT_ZEBRA_CROSSING*/)
    return;
  
  mRunning = true;

  ros_oadrive::ControllerCommand msg;
  msg.drive = true;
  msg.reset = false;
  msg.active_brake = false;
  msg.target_speed = mTargetSpeed;
  mDrivingCommandPub.publish(msg);
}

void StateMachineNode::sendHaltCommand() {
  mRunning = false;

  ros_oadrive::ControllerCommand msg;
  msg.drive = false;
  msg.reset = false;
  msg.active_brake = false;
  msg.target_speed = 0.0f;
  mDrivingCommandPub.publish(msg);
}

void StateMachineNode::sendEmergencyHaltCommand() {
  mRunning = false;

  ros_oadrive::ControllerCommand msg;
  msg.drive = false;
  msg.reset = false;
  msg.active_brake = true;
  msg.target_speed = 0.0f;
  mDrivingCommandPub.publish(msg);
}

void StateMachineNode::publishTransition(StateMachine::State prevState, StateMachine::State newState) {

  ros_oadrive::StateChange msg;

  msg.prevState = static_cast<int>(prevState);
  msg.newState = static_cast<int>(newState);

  mStatePub.publish(msg);
}

void StateMachineNode::timerCallback(const ros::TimerEvent& event) {

  if (!mJuryReady) {
    // Tell them we are ready!
    std_msgs::String responseMsg;
    responseMsg.data = "READY_EVENT";
    mJuryPub.publish(responseMsg);
    mJuryReady = true;
    
    mJuryTimer.stop();
  }

  switch(mCurrentState) {
    case StateMachine::State::FORMING_RESCUE_LANE:
      mStateMachine.triggerEvent(StateMachine::Event::FINISH_FORMING_RESCUE_LANE);
      mEmergencyVehicleTimer.stop();
      break;
    case StateMachine::State::UNFORMING_RESCUE_LANE:
      mStateMachine.triggerEvent(StateMachine::Event::FINISH_UNFORMING_RESCUE_LANE);
      mEmergencyVehicleTimer.stop();
      break;
    case StateMachine::State::SWITCHING_FOR_BYPASS:
      mStateMachine.triggerEvent(StateMachine::Event::FINISH_SWITCHING_FOR_BYPASS);
      mBypassTimer.stop();
      break;
    case StateMachine::State::SWITCHING_BACK_FOR_BYPASS:
      mStateMachine.triggerEvent(StateMachine::Event::FINISH_SWITCHING_BACK_FOR_BYPASS);
      mBypassTimer.stop();
      break;
    case StateMachine::State::PARKING:
    {
      mStateMachine.triggerEvent(StateMachine::Event::FINISH_PARKING);
      std_msgs::String msg;
      msg.data = "MANEUVER_DONE_EVENT";
      mJuryPub.publish(msg);
      mNewManeuver = false;
      mLastManeuverEventRegion = -1;
      sendDriveCommand();
      mParkingTimer.stop();
      break; 
    }
    case StateMachine::State::WAITING_AT_ZEBRA_CROSSING:
    case StateMachine::State::DRIVING_THROUGH_ZEBRA:
      mStateMachine.triggerEvent(StateMachine::Event::LEAVE_ZEBRA_CROSSING);
      ROS_INFO_STREAM("Finished waiting at zebra");
      sendDriveCommand();
      mZebraTimer.stop();
      break;
    case StateMachine::State::WAITING:
      mStateMachine.triggerEvent(StateMachine::Event::STOP_WAITING);
      sendDriveCommand();
      mWaitingTimer.stop();
      break;
    default:
      mJuryTimer.stop();
      mBypassTimer.stop();
      mParkingTimer.stop();
      mEmergencyVehicleTimer.stop();
      mZebraTimer.stop();
      mWaitingTimer.stop();
      break;
  }

  StateMachine::State newState = mStateMachine.queryState();

  if (newState == mCurrentState) {
    ROS_INFO_STREAM("ERROR: No transition taken. THIS SHOULD NEVER HAPPEN BECAUSE IT MEANS THAT A JURY COMMAND IS BEING IGNORED");
    return;
  }

  // special treatment in some cases that have to deal with superstates
  if (mCurrentState == StateMachine::State::UNFORMING_RESCUE_LANE) {
    publishTransition(StateMachine::State::RESCUE_LANE, newState);
  }
  else if (mCurrentState == StateMachine::State::SWITCHING_BACK_FOR_BYPASS) {
    publishTransition(StateMachine::State::BYPASS, newState);
  }
  else {
    publishTransition(mCurrentState, newState);
  }  

  mCurrentState = newState;

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "StateMachineNode");
  ros::NodeHandle nh("~");

  std::string config_folder;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  // Init stuff to get this going.
  oadrive::util::Config::setConfigPath(config_folder, car_name);

  StateMachineNode node(nh);

  ros::spin();
}
