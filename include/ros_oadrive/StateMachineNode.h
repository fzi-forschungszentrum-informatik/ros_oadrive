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
 * \brief Node wrapping the state machine of the car.
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#ifndef ROS_OADRIVE_STATE_MACHINE_NODE_H_
#define ROS_OADRIVE_STATE_MACHINE_NODE_H_


#include <oadrive_world/StateMachine.h>
#include <ros/ros.h>
#include <ros_oadrive/Event.h>
#include <ros_oadrive/StateChange.h>
#include <ros_oadrive/EmergencyBrake.h>
#include <ros_oadrive/EvokeTransition.h>
#include <ros_oadrive/ControllerCommand.h>
#include <ros_oadrive/JuryManeuver.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <time.h>


namespace ros_oadrive {

/*!
  \class StateMachineNode
  \brief Relays events to the state machine and publishes transitions. Also serves for general mission control.

  Subscribes:
    * /aadc/planning/event
*/
class StateMachineNode {

 public:
    StateMachineNode(ros::NodeHandle nh);

 private:

    ros::NodeHandle mNodeHandle;

    StateMachine mStateMachine;

    ros::Publisher mStatePub;
    ros::Publisher mJuryPub;
    ros::Publisher mDrivingCommandPub;
  
    ros::Subscriber mJuryStateSub;
    ros::Subscriber mJuryEventSub;
    ros::Subscriber mEventSub;
    ros::Subscriber mManeuverSub;
    ros::Subscriber mTargetSpeedSub;
    ros::Subscriber mEmergencyBrakeSub;

    ros::Timer mEmergencyVehicleTimer;
    ros::Timer mBypassTimer;
    ros::Timer mParkingTimer;
    ros::Timer mZebraTimer;
    ros::Timer mJuryTimer;
    ros::Timer mWaitingTimer;

    // for tracking current state and publishing new States
    StateMachine::State mCurrentState;

    // tracks emergency brake
    bool mEmergencyBrakeActivated;
    bool mParked;

    // current maneuver
    std::string mCurrentManeuver;

  
    bool mRunning = false;
    bool mJuryStarted = false;
    bool mNewManeuver = false;
    bool mJuryReady = true;
    // Stores the last eventregion used to increment the maneuver count
    // This prevents incrementing the maneuver counter if we start a bypass in an intersection
    // and leave the region while bypassing
    uint64_t mLastManeuverEventRegion = -1;

    int mCrossSectionCounter = 0;

    // current targetSpeed;
    float mTargetSpeed;

    void advertise();
    void subscribe();

    // handles incoming events. main control function for this node
    void eventLogic(const ros_oadrive::Event::ConstPtr& event);

    // handles incoming maneuver commands to pick correct successor state
    void maneuverLogic(const ros_oadrive::JuryManeuver::ConstPtr& msg);

    // handles incoming jury status
    void juryStateLogic(const std_msgs::String::ConstPtr& msg);

    // handles incoming jury events
    void juryEventLogic(const std_msgs::String::ConstPtr& msg);

    // handles incoming target speed
    void targetSpeedLogic(const std_msgs::Float64::ConstPtr& msg);

    // handles emergency brake
    void emergencyBrakeLogic(const ros_oadrive::EmergencyBrake::ConstPtr& msg);

    // publishes state changes
    void publishTransition(StateMachine::State prevState, StateMachine::State newState);

    // functions for sending driving commands to the driver module
    void sendHaltCommand();
    void sendDriveCommand();
    void sendEmergencyHaltCommand();

    // timer callbacks
    void timerCallback(const ros::TimerEvent& event);
};

}  // namespace ros_oadrive

#endif /* ROS_OADRIVE_TRAJECTORY_GENERATOR_H_ */
