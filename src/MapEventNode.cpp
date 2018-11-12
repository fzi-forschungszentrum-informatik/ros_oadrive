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


#include <oadrive_lanedetection/FeatureDetection/StreetTypes.h>
#include <oadrive_lanedetection/RoadPatching/Map.h>
#include <oadrive_world/Environment.h>
#include <oadrive_util/Config.h>

#include <ros/ros.h>
#include "ros_oadrive/MarkerPosition.h"
#include "ros_oadrive/Event.h"
#include "ros_oadrive/JuryManeuver.h"
#include <tf/transform_listener.h>
#include <oadrive_world/StateMachine.h>
#include <ros_oadrive/StateChange.h>

#include <std_msgs/String.h>


using namespace oadrive::util;

using oadrive::lanedetection::OadrivePose;

namespace ros_oadrive
{

class MapEventNode {
private:
    oadrive::lanedetection::Map mMap;
    ros::Subscriber mPoseSubscriber;
    ros::Subscriber mManeuverSub;
    ros::Subscriber mMapSub;
    ros::Subscriber mStateSub;

    ros::Publisher mEventPublisher;
    tf::TransformListener mTfListener;

    bool mMergeManeuver = false;

    bool mApproachingMergeLane = false;
    bool mMerging = false;

    bool mApproachedRamp = false;

    ros_oadrive::Event mLastEvent;

    int mLostCounter = 0;

    // TODO: Enable for the finals
    bool mRampEnabled = false;
    bool mRampExists = false;
    OadrivePose mRampStart;

public:
    MapEventNode(ros::NodeHandle nh)
        : mMap(Config::getString("Map", "Path", "")) {
        
        mPoseSubscriber = nh.subscribe("/aadc/marker_position", 1, &MapEventNode::poseCallback, this);
        mManeuverSub = nh.subscribe("/aadc/jury/current_maneuver", 1000, &MapEventNode::maneuverLogic, this);
        mMapSub = nh.subscribe("/aadc/alpaka_map", 1000, &MapEventNode::mapCallback, this);
        mStateSub = nh.subscribe("/aadc/statemachine/state", 10, &MapEventNode::stateCallback, this);


        mEventPublisher = nh.advertise<ros_oadrive::Event>("/aadc/planning/event", 5);

        // Find ramp in map
        if (mRampEnabled && mMap.hasRamp()) {
            mRampStart = mMap.getRampTakeOff();
            
            ROS_INFO_STREAM("Ramp start is at " << mRampStart.getX() << " " << mRampStart.getY());
        }

        try {
            ROS_INFO("Waiting for transformations to be available..");
            mTfListener.waitForTransform("world", "local", ros::Time(0), ros::Duration(20.0));
            ROS_INFO("..Done!");
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
    }

    void publishMergeEvent() {
        mLastEvent.type = "MERGE_EVENT";

        mEventPublisher.publish(mLastEvent);

        ROS_INFO_STREAM("Approached pull out area! ");
    }

    void publishEntered() {
        mLastEvent.type = "ENTERED_EVENT_REGION";
        mLastEvent.event_region.type = "MERGE_REGION";

        mEventPublisher.publish(mLastEvent);

        ROS_INFO_STREAM("Sent entered!");
    }

    void publishLeft() {
        mLastEvent.type = "LEFT_EVENT_REGION";
        mLastEvent.event_region.type = "MERGE_REGION";

        mEventPublisher.publish(mLastEvent);

        ROS_INFO_STREAM("Sent left!");
    }

    void stateCallback(const StateChange::ConstPtr &msg)
    {
        auto prevState = static_cast<StateMachine::State>(msg->prevState);
        auto newState = static_cast<StateMachine::State>(msg->newState);

        if (prevState == StateMachine::State::ACTIVE || prevState == StateMachine::State::INACTIVE) {
            reset();
        }
    }

    void reset() {
        ROS_INFO_STREAM("RESET!");
        mMergeManeuver = false;
        mApproachedRamp = false;
        mApproachingMergeLane = false;
        mMerging = false;
    }

    void poseCallback(const ros_oadrive::MarkerPosition::ConstPtr &pose_msg)
    {
        try {
            geometry_msgs::PoseStamped localPose;
            localPose.header.frame_id = "local";
            localPose.header.stamp = pose_msg->header.stamp;
            localPose.pose.position.x = pose_msg->pose.x;
            localPose.pose.position.y = pose_msg->pose.y;
            localPose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_msg->pose.theta);

            geometry_msgs::PoseStamped worldPose;
            mTfListener.transformPose("world", ros::Time(0), localPose, "local", worldPose);

            tf::Quaternion q(
                worldPose.pose.orientation.x,
                worldPose.pose.orientation.y,
                worldPose.pose.orientation.z,
                worldPose.pose.orientation.w
            );
            
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            OadrivePose oaPose(worldPose.pose.position.x, worldPose.pose.position.y, yaw);


            // This might be helpful for debugging the ramp in the finals
            // if (mMergeManeuver) {
            //     ROS_INFO_STREAM(oaPose.getX() << " " << oaPose.getY() << " " << mApproachedRamp);
            // }
            
            // Check approaching ramp: (Hacked for the Testevent)
            // if (mMergeManeuver && !mApproachedRamp && oaPose.getX() <= 8.65 && oaPose.getY() > 3.0) {
            if (mRampEnabled && mMergeManeuver && !mApproachedRamp && oaPose.getX() <= mRampStart.getX() && oaPose.getY() > mRampStart.getY()) {
                mApproachedRamp = true;
                
                mLastEvent.type = "APPROACHING_RAMP_EVENT";
                mLastEvent.event_region.type = "";

                mEventPublisher.publish(mLastEvent);
            }

            if ((!mRampEnabled || mApproachedRamp) || mMerging || mApproachingMergeLane) {
                auto approachingMergeLineCenters = mMap.queryMergeLanes(oaPose, 3.5, M_PI/2);
                auto behindMergeLineCenters = mMap.queryMergeLanes(OadrivePose(oaPose.getX(), oaPose.getY(), oaPose.getYaw() + M_PI), 3.0, M_PI/2);

                // Calculate distance
                float distance = 100.0f;
                for (const oadrive::lanedetection::Map::Tile& mergeLane : approachingMergeLineCenters) {
                    distance = std::min(distance, oaPose.distance(mergeLane.pose));
                }
                
                bool approaching = approachingMergeLineCenters.size() > 0;
                bool merging = behindMergeLineCenters.size() > 0 || distance < 0.5;

                if (approaching || merging) {
                    mLostCounter = 0;
                }

                if (mApproachingMergeLane && merging && !mMerging) {
                    publishMergeEvent();

                    mMerging = true;
                    mApproachingMergeLane = false;
                } else if (approaching && !mApproachingMergeLane && !mMerging && mMergeManeuver && (!mRampEnabled || mApproachedRamp)) {
                    // Only active when in merge maneuver
                    publishEntered();

                    mApproachingMergeLane = true;
                    mApproachedRamp = false;
                    
                    for (const oadrive::lanedetection::Map::Tile& mergeLane : approachingMergeLineCenters) {
                        ROS_INFO_STREAM("Approaching merge line, dist: " << oaPose.distance(mergeLane.pose));
                    }
                    
                } else if (!merging && mMerging) {
                    mMerging = false;
                    ROS_INFO_STREAM("Merging finished! ");

                    publishLeft();
                } else if (!approaching && mApproachingMergeLane) {
                    mLostCounter++;
                    ROS_INFO_STREAM("Merge lane lost " << mLostCounter << " times!");

                    if (mLostCounter > 5) {
                        publishLeft();

                        mApproachingMergeLane = false;
                    }
                }
            }      
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
    }

    void maneuverLogic(const ros_oadrive::JuryManeuver::ConstPtr &msg) {
        if (msg->action == "merge_left") {
            mMergeManeuver = true;
        } else {
            mMergeManeuver = false;
        }

        ROS_INFO_STREAM("Next maneuver is merge?: " << mMergeManeuver << " " << msg->action);

        // For testing purposes jury is disabled
        // mMergeManeuver = true;
    }

    void mapCallback(const std_msgs::String::ConstPtr &msg) {
        mMap.loadFromString(msg->data);
    }

};
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "MapEventNode");
  ros::NodeHandle nh("~");

  std::string config_folder;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  // Init stuff to get this going.
  oadrive::util::Config::setConfigPath(config_folder, car_name);

  ros_oadrive::MapEventNode node(nh);

  // Wait for new subscription hits.
  ros::spin();

  return 0;
}