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
 * This node is responsible for switching between tele operation and autonomous driving
 * and gates the correct control commands to the actuators
 *
 */
//----------------------------------------------------------------------


#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ros_oadrive/Event.h>
#include <ros_oadrive/StateChange.h>
#include <oadrive_world/StateMachine.h>


namespace ros_oadrive {

class TeleOperationSwitchNode {
protected:
    ros::Publisher m_pubEvent;
    ros::Publisher m_pubControl;

    ros::Subscriber m_subEvent;
    ros::Subscriber m_subAutonomous;
    ros::Subscriber m_subTeleop;
    ros::Subscriber m_subState;
    
    bool m_teleMode = false;
public:
    int run() {
        // Init ros
        ros::NodeHandle nh("~");
        
        m_pubEvent = nh.advertise<Event>("/aadc/planning/event", 10);
        m_pubControl = nh.advertise<ackermann_msgs::AckermannDrive>("/aadc/control/car", 1);

        m_subAutonomous = nh.subscribe<ackermann_msgs::AckermannDrive>("/aadc/control/car_autonomous", 1, &TeleOperationSwitchNode::callbackAutonomous, this);
        m_subTeleop = nh.subscribe<ackermann_msgs::AckermannDrive>("/aadc/control/car_tele", 1, &TeleOperationSwitchNode::callbackTele, this);
        m_subEvent = nh.subscribe<Event>("/aadc/planning/event", 10, &TeleOperationSwitchNode::callbackEvent, this);

        m_subState = nh.subscribe("/aadc/statemachine/state", 10, &TeleOperationSwitchNode::stateCallback, this);

        ros::spin();

        return 0;
    }

    void callbackAutonomous(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
        if (!m_teleMode) {
            m_pubControl.publish(*msg);
        }
    }

    void callbackTele(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
        if (m_teleMode) {
            m_pubControl.publish(*msg);
        }
    }

    void callbackEvent(const Event::ConstPtr& msg) {
        if (msg->type == "START_TELE_OP_EVENT") {
            m_teleMode = true;
            stop();
            ROS_INFO("SWITCHED TO TELE MODE");
        } else if (msg->type == "END_TELE_OP_EVENT") {
            m_teleMode = false;
            stop();
            ROS_INFO("SWITCHED TO AUTO MODE");
        }
    }

    void reset() {
        m_teleMode = false;
        stop();
        ROS_INFO("RESET!");
    }

    void stop() {
        ackermann_msgs::AckermannDrive msg;
        msg.speed = 0;
        m_pubControl.publish(msg);
    }

    void stateCallback(const StateChange::ConstPtr &msg)
    {
        auto prevState = static_cast<StateMachine::State>(msg->prevState);
        auto newState = static_cast<StateMachine::State>(msg->newState);

        if (prevState == StateMachine::State::ACTIVE || prevState == StateMachine::State::INACTIVE) {
            reset();
        }
    }

};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TeleOperationSwitchNode");
    ros_oadrive::TeleOperationSwitchNode node;
    return node.run();
}
