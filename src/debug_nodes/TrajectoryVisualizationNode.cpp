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
 * Visualizes the Trajectory in RViz.
 * Currently this is somehow time depended, because the starting point of the trajectory depends on the time it arrives at the ControllerNode
 *
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


class TrajectoryVisualizationNode {
protected:
    ros::Publisher mPubRvizTraj;

    oadrive::core::ExtendedPose2d mCurrentCarPoseGlobal;
    oadrive::core::ExtendedPose2d mCarPoseOrigin;
    tf::TransformBroadcaster* m_broadcaster;

public:
    int run() {
        // Init ros
        ros::NodeHandle nh("~");
        mPubRvizTraj = nh.advertise<visualization_msgs::Marker>("/aadc/visualization/trajectory", 1);
        ros::Subscriber subTraj = nh.subscribe("/aadc/planning/trajectory", 1, &TrajectoryVisualizationNode::callbackMultiTrajectory, this);
        ros::Subscriber subPos = nh.subscribe("/aadc/marker_position", 1, &TrajectoryVisualizationNode::callbackMarkerPosition, this);
        ros::Subscriber subLocalPose = nh.subscribe("/DriverModule/LocalCarPose", 1, &TrajectoryVisualizationNode::callbackLocalCarPose, this);


        m_broadcaster = new tf::TransformBroadcaster();


        ros::Rate loop_rate(30);

        // Init controller
        while (ros::ok()) {            
            // Ros lifecycle
            ros::spinOnce();

            tf::Quaternion car_orientation = tf::Quaternion(0, 0, 0, 1);
            car_orientation.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), mCarPoseOrigin.getYaw());
            m_broadcaster->sendTransform(
                tf::StampedTransform(tf::Transform(car_orientation, tf::Vector3(mCarPoseOrigin.getX(), mCarPoseOrigin.getY(), 0.0f)),
                ros::Time::now(),
                "local",
                "trajectory_frame"));

            loop_rate.sleep();
        }

        return 0;
    }

    void publishVisualization(ros_oadrive::MultiTrajectory multiTrajectory)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "trajectory_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 1;

        for (auto &trajectory : multiTrajectory.trajectories) {
            for (auto &trajectoryPoint : trajectory.trajectory) {
                geometry_msgs::Point point;
                point.x = trajectoryPoint.pose.x;
                point.y = trajectoryPoint.pose.y;
                marker.points.push_back(point);
            }
        }
        marker.color.a = 1.0;  // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        mPubRvizTraj.publish(marker);
    }

    void callbackMultiTrajectory(const ros_oadrive::MultiTrajectory &msg)
    {
        mCarPoseOrigin = mCurrentCarPoseGlobal;
        publishVisualization(msg);
    }

    void callbackMarkerPosition(const ros_oadrive::MarkerPosition::ConstPtr& markerPosition_msg){
        mCurrentCarPoseGlobal.setX(markerPosition_msg->pose.x);
        mCurrentCarPoseGlobal.setY(markerPosition_msg->pose.y);
        mCurrentCarPoseGlobal.setYaw(markerPosition_msg->pose.theta);
        mCurrentCarPoseGlobal.setVelocity(markerPosition_msg->speed);
    }

    void callbackLocalCarPose(const ros_oadrive::ExtendedPose2d& localPose)
    {
        tf::Quaternion car_orientation = tf::Quaternion(0, 0, 0, 1);
        car_orientation.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), localPose.pose.theta);
        m_broadcaster->sendTransform(
            tf::StampedTransform(tf::Transform(car_orientation, tf::Vector3(localPose.pose.x, localPose.pose.y, 0.0f)),
            ros::Time::now(),
            "trajectory_frame",
            "trajectory_car"));
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TrajectoryVisualizationNode");
    TrajectoryVisualizationNode rosNode;
    return rosNode.run();
}
