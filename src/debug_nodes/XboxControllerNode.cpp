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
 * Security node - dead man switch using xbox controller
 *
 */
//----------------------------------------------------------------------

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "libgamepad/gamepad.h"

#include <ros_oadrive/Event.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <ros_oadrive/JuryManeuver.h>


class XboxControllerNode {
protected:
    ros::Publisher m_pubJury;
    ros::Publisher m_pubJuryEvent;
    ros::Publisher m_pubManeuver;
    ros::Publisher m_pubEvent;
    ros::Publisher m_pubControl;
    ros::Publisher m_pubControlStamped;
    ros::Subscriber m_subControl;

    ros::ServiceClient m_recorderClient;

    bool m_alive = false;
    bool m_emergency = false;
    bool m_manual = false;

    bool m_btnResetDown = false;
    bool m_btnStartDown = false;
    bool m_btnDpadDown = false;
    bool m_btnShoulderDown = false;
    bool m_btnADown = false;


    ackermann_msgs::AckermannDrive m_lastControl;
public:
    int run() {
        // Init ros
        ros::NodeHandle nh("~");
        m_pubJury = nh.advertise<std_msgs::String>("/aadc/jury/status", 10);
        m_pubJuryEvent = nh.advertise<std_msgs::String>("/aadc/jury/event", 10);
        m_pubManeuver = nh.advertise<ros_oadrive::JuryManeuver>("/aadc/jury/current_maneuver", 10);
        m_pubEvent = nh.advertise<ros_oadrive::Event>("/aadc/planning/event", 10);
        m_pubControl = nh.advertise<ackermann_msgs::AckermannDrive>("/aadc/control/car", 1);
        m_pubControlStamped = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/aadc/control/car_stamped", 1);
        m_subControl = nh.subscribe<ackermann_msgs::AckermannDrive>("/aadc/control/car_raw", 1, &XboxControllerNode::callbackControl, this);

        m_recorderClient = nh.serviceClient<std_srvs::Empty>("/aadc/trajectory_recorder/toggle");

        ros::Rate loop_rate(100);

        // Init controller
        GamepadInit();

        while (ros::ok()) {
            GamepadUpdate();

            gamepadLoop(GAMEPAD_0);
            
            // Ros lifecycle
            ros::spinOnce();
            loop_rate.sleep();
        }

        return 0;
    }

    /**
     * publish zero speed msg
     */
    void stopCar() {
        m_lastControl.speed = 0;
        m_pubControl.publish(m_lastControl);
    }

    void gamepadLoop(GAMEPAD_DEVICE dev) {
        if (!GamepadIsConnected(dev)) {
            ROS_WARN("Controller disconnected!");
            m_alive = false;
            return;
        }

        if (GamepadButtonDown(dev, BUTTON_B)) {
            if (!m_alive) {
                ROS_INFO("Enabling control!");
            }

            m_alive = true;
        } else {
            if (m_alive) {
                stopCar();
                ROS_INFO("Disabling control!");
            }

            m_alive = false;
        }

        // Manual control mode:
        if (GamepadButtonDown(dev, BUTTON_Y)) {
            if (!m_manual) {
                ROS_INFO("Enabling manual mode!");
            }

            m_manual = true;
        } else {
            if (m_manual) {
                stopCar();
                ROS_INFO("Disabling manual mode!");
            }

            m_manual = false;
        }

        if (m_manual) {
            // speed
            float right = GamepadTriggerLength(dev, TRIGGER_RIGHT); // Forward
            float left = GamepadTriggerLength(dev, TRIGGER_LEFT); // Backward

            float speed = right - left;

            // Steering angle
            float lx, ly;
            float length = GamepadStickLength(dev, STICK_LEFT);
            GamepadStickNormXY(dev, STICK_LEFT, &lx, &ly);

            ackermann_msgs::AckermannDrive control;
            ackermann_msgs::AckermannDriveStamped controlStamped;
            // control.header = std_msgs::Header();
            // control.header.stamp = ros::Time::now();
            control.speed = speed;
            controlStamped.drive.speed = control.speed;
            //control.steering_angle = M_PI / 2.f * lx * length;
            control.steering_angle = 0.5f * lx * length;
            controlStamped.drive.steering_angle = control.steering_angle;
            controlStamped.header.stamp = ros::Time::now();

            m_pubControlStamped.publish(controlStamped);
            m_pubControl.publish(control);
        }

        // Jury Mode
        if (GamepadButtonDown(dev, BUTTON_BACK)) {
            if (!m_btnResetDown) {
                // Send reset
                std_msgs::String juryMsg;
                juryMsg.data = "SectorReset";
                m_pubJury.publish(juryMsg);

                std_msgs::String juryEvent;
                juryEvent.data = "GET_READY_EVENT";
                m_pubJuryEvent.publish(juryEvent);

                /*ros_oadrive::Event eventMsg;
                eventMsg.type = "RESET";
                m_pubEvent.publish(eventMsg);*/

                ROS_INFO("Sent reset!");
                m_btnResetDown = true;
            }
        } else {
            m_btnResetDown = false;
        }
        if (GamepadButtonDown(dev, BUTTON_START)) {
            if (!m_btnStartDown) {
                // Send start
                std_msgs::String juryMsg;
                juryMsg.data = "Start";
                m_pubJury.publish(juryMsg);

                std_msgs::String juryEvent;
                juryEvent.data = "START_EVENT";
                m_pubJuryEvent.publish(juryEvent);

                ROS_INFO("Sent start!");
                m_btnStartDown = true;
            }
        } else {
            m_btnStartDown = false;
        }

        if (GamepadButtonDown(dev, BUTTON_A)) {
            if (!m_btnADown) {
                 std_srvs::Empty srv;
                if (m_recorderClient.call(srv))
                    ROS_INFO_STREAM("Toggling trajectory recording");
                else
                    ROS_INFO_STREAM("Failed to toggle trajectory recording");

                m_btnADown = true;
            }
        }
        else {
            m_btnADown = false;
        }

        if (GamepadButtonDown(dev, BUTTON_DPAD_UP) && !GamepadButtonDown(dev, BUTTON_DPAD_LEFT) && !GamepadButtonDown(dev, BUTTON_DPAD_RIGHT)) {
            if (!m_btnDpadDown) {
                ros_oadrive::JuryManeuver maneuverMsg;
                maneuverMsg.action = "straight";
                m_pubManeuver.publish(maneuverMsg);

                ROS_INFO("Sent straight!");
                m_btnDpadDown = true;
            }
        } else if (GamepadButtonDown(dev, BUTTON_DPAD_LEFT)) {
            if (!m_btnDpadDown) {
                ros_oadrive::JuryManeuver maneuverMsg;
                maneuverMsg.action = "left";
                m_pubManeuver.publish(maneuverMsg);

                ROS_INFO("Sent left!");
                m_btnDpadDown = true;
            }
        } else if (GamepadButtonDown(dev, BUTTON_DPAD_RIGHT)) {
            if (!m_btnDpadDown) {
                ros_oadrive::JuryManeuver maneuverMsg;
                maneuverMsg.action = "right";
                m_pubManeuver.publish(maneuverMsg);

                ROS_INFO("Sent right!");
                m_btnDpadDown = true;
            }
        } else {
            m_btnDpadDown = false;
        }

        // if (GamepadButtonDown(dev, BUTTON_X)) {
        //     if (!m_emergency) {
        //         ROS_INFO("Emergency actived");
        //         ros_oadrive::Event event;
        //         event.type = "ENTERED_EVENT_REGION";
        //         event.event_region.type = "EMERGENCY_VEHICLE_REGION";
        //         m_pubEvent.publish(event);
        //     }

        //     m_emergency = true;
        // } else {
        //     if (m_emergency) {
        //         ROS_INFO("Emergency deactived");
        //         ros_oadrive::Event event;
        //         event.type = "LEFT_EVENT_REGION";
        //         event.event_region.type = "EMERGENCY_VEHICLE_REGION";
        //         m_pubEvent.publish(event);
        //     }

        //     m_emergency = false;
        // }


        // Pullouts
        // if (GamepadButtonDown(dev, BUTTON_LEFT_SHOULDER)) {
        //     if (!m_btnShoulderDown) {
        //         ros_oadrive::JuryManeuver maneuverMsg;
        //         maneuverMsg.action = "pull_out_left";
        //         m_pubManeuver.publish(maneuverMsg);

        //         ros_oadrive::Event eventMsg;
        //         eventMsg.type = "PULL_OUT_LEFT_EVENT";
        //         m_pubEvent.publish(eventMsg);

        //         ROS_INFO("Sent pull_out_left!");
        //         m_btnShoulderDown = true;
        //     }
        // } else if (GamepadButtonDown(dev, BUTTON_RIGHT_SHOULDER)) {
        //     if (!m_btnShoulderDown) {
        //         ros_oadrive::JuryManeuver maneuverMsg;
        //         maneuverMsg.action = "pull_out_right";
        //         m_pubManeuver.publish(maneuverMsg);

        //         ros_oadrive::Event eventMsg;
        //         eventMsg.type = "PULL_OUT_RIGHT_EVENT";
        //         m_pubEvent.publish(eventMsg);

        //         ROS_INFO("Sent pull_out_right!");
        //         m_btnShoulderDown = true;
        //     }
        // } else {
        //     m_btnShoulderDown = false;
        // }
    }

    void callbackControl(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
        if (!m_manual) {
            m_lastControl = *msg;

            if (!m_alive) {
                m_lastControl.speed = 0;
            }
            m_pubControl.publish(m_lastControl);
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "XboxControllerNode");
    XboxControllerNode rosNode;
    return rosNode.run();
}
