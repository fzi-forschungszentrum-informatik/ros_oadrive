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
 */
//----------------------------------------------------------------------


#include "ros/ros.h"
#include "ackermann_msgs/AckermannDrive.h"
// #include "debug_nodes/libgamepad/gamepad.h"
#include <ros_oadrive/Event.h>
#include "icl_hardware_g25_wheel/G25RacingWheel.h"


using namespace icl_hardware::g25;


class GameControllerNode : public G25RacingWheel {
protected:
    ros::Publisher m_pubEvent;
    ros::Publisher m_pubControl;

    float m_steering = 0;
    float m_speedForward = 0;
    float m_speedBackward = 0;

    ackermann_msgs::AckermannDrive m_lastControl;
public:
    void buttonPress(G25RacingWheel::ButtonType btn, G25RacingWheel::ButtonPress press)
    {
        if ((btn == G25RacingWheel::ButtonType::WheelButonLeft || btn == G25RacingWheel::ButtonType::WheelButtonRight)
            && press == G25RacingWheel::ButtonPress::Down) {
            ROS_INFO("END_TELE_OP_EVENT sent!");
            ros_oadrive::Event msg;
            msg.type = "END_TELE_OP_EVENT";

            m_pubEvent.publish(msg);
        }
    }

    void steer(double angle)
    {
        m_steering = angle * M_PI / 180.0 * 0.3; // * a factor to get better results
        // ROS_INFO_STREAM("Steering to " << m_steering);
    }

    void accelerate(double value)
    {
        m_speedForward = value;
    }

    void brake(double value)
    {
        m_speedBackward = value;
    }


    int run() {
        // Init ros
        ros::NodeHandle nh("~");
        m_pubEvent = nh.advertise<ros_oadrive::Event>("/aadc/planning/event", 2);
        m_pubControl = nh.advertise<ackermann_msgs::AckermannDrive>("/aadc/control/car_tele", 2);

        ros::Rate loop_rate(25);

        while (ros::ok()) {
            this->readRacingWheel();

            m_lastControl.speed = (m_speedForward - m_speedBackward) * 0.5;
            m_lastControl.steering_angle = m_steering;

            m_pubControl.publish(m_lastControl);
            
            // Ros lifecycle
            ros::spinOnce();
            loop_rate.sleep();
        }

        return 0;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GameControllerNode");
    GameControllerNode rosNode;
    return rosNode.run();
}
