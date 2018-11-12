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
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include <ros_oadrive/LightControlNode.h>

using namespace ros_oadrive;

LightControlNode::LightControlNode(ros::NodeHandle nh) {

  mNodeHandle = nh;

  mIsInactive = true;

  mHeadLight = false;
  mBrakeLight = false;
  mHazardLight = false;
  mReverseLight = false;
  mTurnLeftLight = false;
  mTurnRightLight = false;

  mCurrentState = StateMachine::State::INACTIVE;

  advertise();
  subscribe();
}

void LightControlNode::advertise() {
  mLightPublisher = mNodeHandle.advertise<ros_oadrive::LightStatus>("/aadc/control/light_status", 10);
}

void LightControlNode::subscribe() {
  mStateSubscriber = mNodeHandle.subscribe("/aadc/statemachine/state", 10, &LightControlNode::stateCallback, this);
  mControlSubscriber = mNodeHandle.subscribe("/aadc/control/car", 1, &LightControlNode::controlCallback, this);
  mEmergencyBrakeSubscriber = mNodeHandle.subscribe("/aadc/statemachine/emergency_brake", 1, &LightControlNode::emergencyBrakeCallback, this);
}

void LightControlNode::stateCallback(const StateChange::ConstPtr &msg) {
  auto prevState = static_cast<StateMachine::State>(msg->prevState);
  auto newState = static_cast<StateMachine::State>(msg->newState);

  mCurrentState = newState;

  // if reset was performed, also reset all lights
  if (prevState == StateMachine::State::ACTIVE) {
    mHeadLight = true;
    mBrakeLight = false;
    mHazardLight = false;
    mReverseLight = false;
    mTurnLeftLight = false;
    mTurnRightLight = false;
  }

  // handle entered states
  switch(newState) {
    case StateMachine::INACTIVE:
      mHeadLight = false;
      mIsInactive = true;
      break;
    case StateMachine::IDLING:
      mIsInactive = false;
      mHeadLight = true;
      mBrakeLight = false;
      mHazardLight = false;
      mReverseLight = false;
      mTurnLeftLight = false;
      mTurnRightLight = false;
      break;
    case StateMachine::PARKING:
    case StateMachine::WAITING:
    case StateMachine::TELE_OPERATION:
      mHazardLight = true;
      break;
    case StateMachine::PULLING_OUT_LEFT:
    case StateMachine::PARKING_LEFT:
    case StateMachine::TURNING_LEFT:    
    case StateMachine::CURRENTLY_MERGING:
      mTurnLeftLight = true;
      break;
    case StateMachine::PULLING_OUT_RIGHT:
    case StateMachine::PARKING_RIGHT:
    case StateMachine::TURNING_RIGHT:    
      mTurnRightLight = true;
      break;
    case StateMachine::SLOW_DRIVING:
    case StateMachine::SLOW_CAR_FOLLOWING:
      mBrakeLight = true;
      break;
    // no need to turn off turning signals between states, because adtf lights work in a weird way
    case StateMachine::FORMING_RESCUE_LANE:
      mTurnRightLight = true;
      break;
    case StateMachine::WAITING_IN_RESCUE_LANE:
      mHazardLight = true;
      break;
    case StateMachine::UNFORMING_RESCUE_LANE:
      mTurnLeftLight = true;
      break;
    // light logic for bypass, similar to rescue lane
    case StateMachine::SWITCHING_FOR_BYPASS:
      mTurnLeftLight = true;
      break;
    case StateMachine::CURRENTLY_BYPASSING:
      mTurnLeftLight = false;
      break;
    case StateMachine::SWITCHING_BACK_FOR_BYPASS:
      mTurnRightLight = true;
      break;
    case StateMachine::WAITING_AT_ZEBRA_CROSSING:
      mBrakeLight = true;
      break;
    default:
      break;
  }

  // handle left states
  switch(prevState) {
  	case StateMachine::INACTIVE:
  		mHeadLight = true;
  		mIsInactive = false;
  		break;
    case StateMachine::PARKING:
    case StateMachine::WAITING:
    case StateMachine::TELE_OPERATION:
      mTurnLeftLight = false;
      mTurnRightLight = false;
      mHazardLight = false;
      break;
    case StateMachine::TURNING_LEFT:
    case StateMachine::PULLING_OUT_LEFT:
    case StateMachine::MERGING:
      mTurnLeftLight = false;
      break;
    case StateMachine::TURNING_RIGHT:
    case StateMachine::PULLING_OUT_RIGHT:
      mTurnRightLight = false;
      break;
    case StateMachine::SLOW_DRIVING:
    case StateMachine::SLOW_CAR_FOLLOWING:
      mBrakeLight = false;
      break;
    case StateMachine::RESCUE_LANE:
      mTurnLeftLight = false;
      mTurnRightLight = false;
      mHazardLight = false;
      break;
    case StateMachine::BYPASS:
      mTurnLeftLight = false;
      mTurnRightLight = false;
      mHazardLight = false;
      break;
    case StateMachine::WAITING_AT_ZEBRA_CROSSING:
      mBrakeLight = false;
      break;
    default:
      break;
  }

  publishLightMessage();
}

void LightControlNode::controlCallback(const ackermann_msgs::AckermannDrive msg) {
  float speed = msg.speed;

  // check direction of speed to see whether car is driving in reverse

  if (speed < 0) {
    //if (!mIsInactive)
      mReverseLight = true;
  }
  else {
    mReverseLight = false;
  }

  publishLightMessage();
}

void LightControlNode::emergencyBrakeCallback(const ros_oadrive::EmergencyBrake::ConstPtr& msg) {
  bool em_state = msg->emergency_brake;

  // if emergency brake is active, activate brake lights
  if (mCurrentState == StateMachine::WAITING_AT_ZEBRA_CROSSING ||
      mCurrentState == StateMachine::WAITING_IN_RESCUE_LANE)
      return;
      
  if (em_state) {
    if (!mIsInactive && (mCurrentState == StateMachine::NORMAL_DRIVING || mCurrentState == StateMachine::SLOW_DRIVING))
      mBrakeLight = true;
  }
  else {
    mBrakeLight = false;
  }
}

void LightControlNode::publishLightMessage() {
  ros_oadrive::LightStatus lightMsg;

  lightMsg.headLightsOn = mHeadLight;
  lightMsg.breakingLightsOn = mBrakeLight;
  lightMsg.hazardLightsOn = mHazardLight;
  lightMsg.reverseLightsOn = mReverseLight;
  lightMsg.turnLeftLightsOn = mTurnLeftLight;
  lightMsg.turnRightLightsOn = mTurnRightLight;

  mLightPublisher.publish(lightMsg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "LightControlNode");
  ros::NodeHandle nh("~");

  LightControlNode node(nh);

  ros::spin();
}