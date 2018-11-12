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
 * Very bad position faker, just to test somethings without a car - this does NOT behave like the car (Not even a little bit)
 */
//----------------------------------------------------------------------

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros_oadrive/ControllerCommand.h>
#include <opencv2/highgui.hpp>
#include <ros_oadrive/MultiTrajectory.h>
#include <ros_oadrive/ControllerStatus.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <visualization_msgs/Marker.h>
#include <ros_oadrive/MultiTrajectoryConverter.h>
#include <ros_oadrive/MarkerPosition.h>


using namespace ros_oadrive;
using namespace oadrive::world;
using namespace oadrive::core;


class PositionFakerNode {
protected:
    ros::Publisher mPubMarkerPos;
    tf::TransformBroadcaster* m_broadcaster;

    ros_oadrive::MarkerPosition mCurrentPose;
    ackermann_msgs::AckermannDrive mLastControl;
public:
    int run() {
        // Init ros
        ros::NodeHandle nh("~");
        mPubMarkerPos = nh.advertise<ros_oadrive::MarkerPosition>("/aadc/marker_position", 1);
        auto ackermannSub = nh.subscribe("/aadc/control/car", 1, &PositionFakerNode::callbackAckermann, this);
        m_broadcaster = new tf::TransformBroadcaster();

        ros::Rate loop_rate(30);
    
        const float dt = 1.0/30.0;

        mCurrentPose.pose.x = 0;
        mCurrentPose.pose.y = 0;
        mCurrentPose.pose.theta = 0;
        mCurrentPose.speed = 0;

        // Init controller
        while (ros::ok()) {
            // Ros lifecycle
            ros::spinOnce();

            // Interpolate
            if (mLastControl.speed > mCurrentPose.speed) {
                mCurrentPose.speed += dt * (mLastControl.speed - mCurrentPose.speed); // To "simulate" some speedup
            } else {
                 mCurrentPose.speed += dt * 5 * (mLastControl.speed - mCurrentPose.speed); // To "simulate" breaking
            }

            float travelledDist = dt * mCurrentPose.speed;
            mCurrentPose.pose.x += cos(mCurrentPose.pose.theta) * travelledDist;
            mCurrentPose.pose.y += sin(mCurrentPose.pose.theta) * travelledDist;
            mCurrentPose.pose.theta -= dt * 5 * mCurrentPose.speed * mLastControl.steering_angle; // Again: Very stupid faker, but it does its job

            if (mCurrentPose.pose.theta > M_PI) {
                mCurrentPose.pose.theta -= 2 * M_PI;
            } else if (mCurrentPose.pose.theta < -1 * M_PI) {
                mCurrentPose.pose.theta += 2 * M_PI;
            }
            
            // TODO: add some noise before sending?

            // Send position
            mCurrentPose.header.stamp = ros::Time::now();
            mPubMarkerPos.publish(mCurrentPose);

            // Send as tf
            tf::Quaternion car_orientation = tf::Quaternion(0, 0, 0, 1);
            m_broadcaster->sendTransform(
                tf::StampedTransform(tf::Transform(car_orientation, tf::Vector3(mCurrentPose.pose.x, mCurrentPose.pose.y, 0.0f)),
                ros::Time::now(),
                "world",
                "car"));
            car_orientation.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), mCurrentPose.pose.theta);
            m_broadcaster->sendTransform(
                tf::StampedTransform(tf::Transform(car_orientation, tf::Vector3(0.0f, 0.0f, 0.0f)),
                ros::Time::now(),
                "car",
                "oriented_car"));

            loop_rate.sleep();
        }

        return 0;
    }

    void callbackAckermann(const ackermann_msgs::AckermannDrive& msg)
    {
        mLastControl.steering_angle = msg.steering_angle;
        mLastControl.speed = msg.speed;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PositionFakerNode");
    PositionFakerNode rosNode;
    return rosNode.run();
}
