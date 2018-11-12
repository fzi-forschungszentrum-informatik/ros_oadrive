// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

/*!\file
 * \brief Node controlling the lights of the car.
 *
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
*/

#ifndef ROS_OADRIVE_LIGHT_CONTROL_NODE_H_
#define ROS_OADRIVE_LIGHT_CONTROL_NODE_H_


#include <oadrive_world/StateMachine.h>
#include <ros/ros.h>
#include <ros_oadrive/StateChange.h>
#include <ros_oadrive/LightStatus.h>
#include <ros_oadrive/EmergencyBrake.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <iostream>
#include <time.h>

class LightControlNode {

public:
    LightControlNode(ros::NodeHandle nh);

private:
    ros::NodeHandle mNodeHandle;

    void subscribe();
    void advertise();

    // calback for car control subscriber
    void controlCallback(const ackermann_msgs::AckermannDrive msg);

    // callback for state subscriber
    void stateCallback(const ros_oadrive::StateChange::ConstPtr &msg);

    // callback for emergency brake subscriber
    void emergencyBrakeCallback(const ros_oadrive::EmergencyBrake::ConstPtr& msg);
    
    // builds and publishes a light message representing the current light state
    void publishLightMessage();

    ros::Subscriber mControlSubscriber;
    ros::Subscriber mStateSubscriber;
    ros::Subscriber mEmergencyBrakeSubscriber;

    ros::Publisher mLightPublisher;

    StateMachine::State mCurrentState;

    // variables representing the current light state
    bool mHeadLight;
    bool mBrakeLight;
    bool mHazardLight;
    bool mReverseLight;
    bool mTurnLeftLight;
    bool mTurnRightLight;

    bool mIsInactive;
};

#endif