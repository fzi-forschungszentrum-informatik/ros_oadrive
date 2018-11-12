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


#include <queue>

#include <oadrive_util/Config.h>
#include <oadrive_util/Navigator.h>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_world/StateMachine.h>
#include <oadrive_obstacle/ObjectTracker.h>

#include <ros/ros.h>
#include <ros_oadrive/TrackedObjectsConverter.h>

#include <ros_oadrive/JuryManeuver.h>
#include <ros_oadrive/Event.h>
#include <ros_oadrive/MarkerPosition.h>
#include <ros_oadrive/StateChange.h>
#include <ros_oadrive/StreetPatchOccupancy.h>
#include <ros_oadrive/TrackedObjects.h>
#include <ros_oadrive/StreetPatch.h>    
#include <ros_oadrive/StreetPatchList.h>
#include <ros_oadrive/NavObstruction.h>

#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

using namespace oadrive::util;

namespace ros_oadrive
{

class NavigatorNode {
private:
    oadrive::util::Navigator mNavigator;
    std::list<oadrive::util::Navigator::Maneuver> mManeuverList;
    std::list<oadrive::core::ExtendedPose2d> mWayPoints;
    std::list<std::tuple<oadrive::core::ExtendedPose2d, float, float>> mObstructedTiles;

    ros::Publisher mManeuverPub;
    ros::Publisher mEventPub;
    ros::Publisher mJuryPub;
    ros::Publisher mNavGoalPub;
    ros::Publisher mPubRvizPath;
    ros::Publisher mPubRvizObstructions;
    ros::Publisher mPlanningEventPub;
    ros::Publisher mPubRvizBusStops;
    ros::Publisher mNavObsPub;

    ros::Subscriber mEventSub;
    ros::Subscriber mPlanningEventSub;
    ros::Subscriber mCarPoseSub;
    ros::Subscriber mNavGoalSub;
    ros::Subscriber mRoadObstructionSub;
    ros::Subscriber mStateSub;
    ros::Subscriber mPatchOccupancySub;
    ros::Subscriber mTrackedObjectsSub;
    ros::Subscriber mPatchSub;
    ros::Subscriber mNavObsSub;

    ros::Timer mReplanTimer;

    tf::TransformListener mTfListener;

    bool mRunning = false;
    bool mCurrentlyOnMission = false;

    oadrive::core::ExtendedPose2d mLocalPose;
    oadrive::core::ExtendedPose2d mCurrentPose;
    oadrive::core::ExtendedPose2d mTargetPose;

    StateMachine::State mCurrentState;

    bool mTaxiMode = false;
    bool mBusMode = true;
    bool mPoseInitialized = false;
    bool mTeleOpMode = false;

    std::set<unsigned int> mSeenPatchIDs;

public:

    void reset() {
        mSeenPatchIDs.clear();
        mNavigator.reset();
        mTeleOpMode = false;
    }

    bool generateNewPath(bool startIsEndAllowed = true) {
        return mNavigator.generateManeuverList(mCurrentPose, mTargetPose, mManeuverList, mWayPoints, startIsEndAllowed);
    }

    bool generateRandomNewPath() {
        bool success = false;

        do {
            mTargetPose = mNavigator.generateRandomTarget();
            success = generateNewPath();
        } while (!success);

        if (!mManeuverList.empty()) {
            publishCurrentManeuver();
        }
        
        return success;
    }

    bool generateBusPath() {
        bool success = false;

        mTargetPose = mNavigator.getNextBusStopTarget();

        success = generateNewPath(false);

        if (!mManeuverList.empty()) {
            publishCurrentManeuver();
        }

        return success;
    }

    void publishGetReady() {
        std_msgs::String event;
        event.data = "GET_READY_EVENT";
        mEventPub.publish(event);

        mRunning = true;
    }

    void publishStop() {
        std_msgs::String event;
        event.data = "STOP_EVENT";
        mEventPub.publish(event);

        // TODO: Remove as soon as rewritten in ros_oadrive
        // Temporarily also publish to jury status
        std_msgs::String juryMsg;
        juryMsg.data = "Stop";
        mJuryPub.publish(juryMsg);

        mRunning = false;
    }

    void publishStart() {
        std_msgs::String event;
        event.data = "START_EVENT";
        mEventPub.publish(event);

        // TODO: Remove as soon as rewritten in ros_oadrive
        // Temporarily also publish to jury status
        std_msgs::String juryMsg;
        juryMsg.data = "Start";
        mJuryPub.publish(juryMsg);
    }   

    bool incrementManeuver() {
        if (mManeuverList.empty()) {
            std::cout << "ERROR: Maneuver list is empty." << std::endl;
            return false;
        }
        mManeuverList.pop_front();
        mNavigator.finishManeuver();
        if (mManeuverList.empty()) {
            std::cout << "Maneuver list is finished." << std::endl;
            return false;
        }
        return true;
    }

    void publishCurrentManeuver() {
        auto current = mManeuverList.front();
        ros_oadrive::JuryManeuver msg;
        msg.id = -1;
        msg.action = current.name;
        msg.extra = current.extra;

        mManeuverPub.publish(msg);
    }

    void juryEventCallback(const std_msgs::String::ConstPtr& event) {
        std::string type = event->data;

        if (type == "READY_EVENT") {
            if (!mManeuverList.empty()) {
                publishCurrentManeuver();
            }
            else {
                mManeuverList.push_back(oadrive::util::Navigator::Maneuver("cross_parking", -1));
                publishCurrentManeuver();
            }
            mRunning = true;
        }
        else if (type == "STOP_EVENT") {
            mRunning = false;
        } else if (type == "MANEUVER_DONE_EVENT") {
            if (mRunning) {
                // Increment Maneuver list and send new maneuver
                if (incrementManeuver()) {
                    publishCurrentManeuver();
                } 
            }
        }
    }

    void publishVisualization()
    {
        // path
        if (mRunning && ((!mTaxiMode && !mBusMode)|| ((mTaxiMode || mBusMode) && !mCurrentlyOnMission)))
        {
            // ROS_INFO_STREAM("VISUALIZING NOT ON MISSION");
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 1;

            geometry_msgs::Point point;
            point.x = mCurrentPose.getX();
            point.y = mCurrentPose.getY();
            marker.points.push_back(point);

            for (auto& wayPoint : mWayPoints) {
                geometry_msgs::Point point;
                point.x = wayPoint.getX();
                point.y = wayPoint.getY();
                marker.points.push_back(point);
            }

            geometry_msgs::Point point2;
            point2.x = mTargetPose.getX();
            point2.y = mTargetPose.getY();
            marker.points.push_back(point2);

            marker.color.a = 1.0;  // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            mPubRvizPath.publish(marker);

            visualization_msgs::Marker target;
            target.header.frame_id = "world";
            target.header.stamp = ros::Time::now();
            target.ns = "";
            target.id = 1;
            target.type = visualization_msgs::Marker::CYLINDER;
            target.action = visualization_msgs::Marker::DELETE;
            mPubRvizPath.publish(target);
        }
    
        int id = 1;
        // obstructed tiles
        for (auto& t : mObstructedTiles) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            marker.ns = "";
            marker.id = id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(1.0);
            marker.pose.position.x = (std::get<0>(t)).getX();
            marker.pose.position.y = (std::get<0>(t)).getY();
            marker.pose.position.z = 0;

            marker.scale.x = (std::get<1>(t));
            marker.scale.y = (std::get<2>(t));
            marker.scale.z = 0.01;

            marker.color.a = 0.9;  // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            mPubRvizObstructions.publish(marker);
            id++;
        }

        // mission target
        if ((mTaxiMode || mBusMode) && mCurrentlyOnMission) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 1;

            geometry_msgs::Point point;
            point.x = mCurrentPose.getX();
            point.y = mCurrentPose.getY();
            marker.points.push_back(point);

            for (auto& wayPoint : mWayPoints) {
                geometry_msgs::Point point;
                point.x = wayPoint.getX();
                point.y = wayPoint.getY();
                marker.points.push_back(point);
            }

            geometry_msgs::Point point2;
            point2.x = mTargetPose.getX();
            point2.y = mTargetPose.getY();
            marker.points.push_back(point2);

            marker.color.a = 1.0;  // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            mPubRvizPath.publish(marker);

            // visualize target
            visualization_msgs::Marker target;
            target.header.frame_id = "world";
            target.header.stamp = ros::Time::now();
            target.ns = "";
            target.id = 1;
            target.type = visualization_msgs::Marker::CYLINDER;
            target.action = visualization_msgs::Marker::ADD;
            target.pose.position.x = mTargetPose.getX();
            target.pose.position.y = mTargetPose.getY();
            target.scale.x = 0.4;
            target.scale.y = 0.4;
            target.scale.z = 0.01;
            target.color.a = 1.0;  // Don't forget to set the alpha!
            target.color.r = 1.0;
            target.color.g = 0.0;
            target.color.b = 1.0;
            mPubRvizPath.publish(target);
        }

        // bus stops
        if (mBusMode && mCurrentlyOnMission) {
            std::list<oadrive::core::ExtendedPose2d> busStops;
            mNavigator.getRemainingBusStops(busStops);
            
            int busStopID = 0;
            for (auto& busStop : busStops) {
                visualization_msgs::Marker stopMarker;
                stopMarker.header.frame_id = "world";
                stopMarker.header.stamp = ros::Time();
                stopMarker.ns = "";
                stopMarker.id = busStopID;
                stopMarker.type = visualization_msgs::Marker::CYLINDER;
                stopMarker.action = visualization_msgs::Marker::ADD;
                stopMarker.lifetime = ros::Duration(2.0);
                stopMarker.pose.position.x = busStop.getX();
                stopMarker.pose.position.y = busStop.getY();
                stopMarker.pose.position.z = 0;

                stopMarker.scale.x = 0.25;
                stopMarker.scale.y = 0.25;
                stopMarker.scale.z = 0.01;

                stopMarker.color.a = 0.9;  // Don't forget to set the alpha!
                stopMarker.color.r = 1.0;
                stopMarker.color.g = 1.0;
                stopMarker.color.b = 0.0;
                mPubRvizBusStops.publish(stopMarker);
                busStopID++;
            }
        }
    }

    void carPoseCallback(const ros_oadrive::MarkerPosition::ConstPtr &pose_msg)
    {
        bool goalReached = false;

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

            mLocalPose = oadrive::core::ExtendedPose2d(localPose.pose.position.x, localPose.pose.position.y, pose_msg->pose.theta);
            mCurrentPose = oadrive::core::ExtendedPose2d(worldPose.pose.position.x, worldPose.pose.position.y, yaw);
            mPoseInitialized = true;

        } catch (tf::TransformException ex) {
            mCurrentPose = oadrive::core::ExtendedPose2d(0,0,0);
            ROS_ERROR("%s",ex.what());
        }
        
        if (!mWayPoints.empty()) {
            oadrive::core::ExtendedPose2d currentWaypoint = mWayPoints.front();

            float distanceToWP = sqrt(pow(mCurrentPose.getX()-currentWaypoint.getX(),2.f) +
                                    pow(mCurrentPose.getY()-currentWaypoint.getY(),2.f));

            if (distanceToWP <= 0.8f) {
                mWayPoints.pop_front();
            }
        }

        float distanceToTarget = sqrt(pow(mCurrentPose.getX()-mTargetPose.getX(),2.f) +
                                      pow(mCurrentPose.getY()-mTargetPose.getY(),2.f));

        if (distanceToTarget <= 0.8) {
            std::cout << "REACHED GOAL" << std::endl;
            mWayPoints.clear(); 
            goalReached = true;
        }
        else if ((distanceToTarget <= 0.5) && (mManeuverList.size() == 1) && (mManeuverList.front().name == "merge_left")) {
            std::cout << "REACHED GOAL ON MERGE LANE" << std::endl;
            mWayPoints.clear();  
            goalReached = true;
        }

        if (!mTaxiMode && !mBusMode) {
            publishStop();
        }

        if (mTaxiMode && mCurrentlyOnMission && goalReached) {
            mCurrentlyOnMission = false;

            ROS_INFO_STREAM("Waiting for Person to get out...");
            ros_oadrive::Event msg;
            msg.type = "START_WAITING_EVENT";
            mPlanningEventPub.publish(msg);
        }

        if (mBusMode && mCurrentlyOnMission && goalReached) {
            ROS_INFO_STREAM("Waiting for people to enter bus...");
            ros_oadrive::Event msg;
            msg.type = "START_WAITING_EVENT";
            mPlanningEventPub.publish(msg);
        }

        // generate random new target
        if (goalReached && mTaxiMode && !mCurrentlyOnMission) {
            generateRandomNewPath();

            publishStart();
        }

        int termCounter = 0;

        if (mBusMode && !mCurrentlyOnMission) {
            mCurrentlyOnMission = true;
            generateBusPath();
            //publishGetReady();
            //reset();
            //sleep(2);
            //publishStart();
            termCounter--;
        }

        // generate next bus stop Target
        if (goalReached && mBusMode && mCurrentlyOnMission) {
            while(!generateBusPath()) {
                termCounter++;
                if (termCounter >= mNavigator.getNumberOfBusStops()-1) {
                    break;
                }
            }
        }

        publishVisualization();
    }

    void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {

        /*if (mNavigator.isCloseToIllegalTile(mCurrentPose)) {
            geometry_msgs::PoseStamped resend;
            resend.header = pose_msg->header;
            resend.pose = pose_msg->pose;
            mNavGoalPub.publish(resend);
            return;
        }

        publishStop();
    
        tf::Quaternion q(
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z,
            pose_msg->pose.orientation.w
        );

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        mTargetPose = oadrive::core::ExtendedPose2d(pose_msg->pose.position.x, pose_msg->pose.position.y, yaw);
        
        if (!generateNewPath()) {
            return;
        }
        
        mCurrentlyOnMission = false;*/

        publishGetReady();
        reset();
        sleep(2);
        publishStart();
    }

    void publishObstruction(oadrive::core::ExtendedPose2d& worldPoint, bool obstructed) {
        NavObstruction msg;
        msg.world_pose.x = worldPoint.getX();
        msg.world_pose.y = worldPoint.getY();
        msg.obstructed = obstructed;

        mNavObsPub.publish(msg);
    }

    void subObstruction(const ros_oadrive::NavObstruction::ConstPtr &msg) {
        oadrive::core::ExtendedPose2d point(msg->world_pose.x, msg->world_pose.y, 0);

        publishVisualization();

        // toggle if the values are different
        if (mNavigator.isObstructed(point) != msg->obstructed) {
            
            ROS_INFO_STREAM("Obstructing tile at " << point.getX() << "," << point.getY() << ", external: " << msg->external_observations << "!");
            
            if (!mNavigator.toggleRoadObstruction(mTargetPose, mCurrentPose, point, mObstructedTiles)) {
                ros_oadrive::Event msg;
                msg.type = "START_TELE_OP_EVENT";
                mPlanningEventPub.publish(msg);
            }

            if (!replan()) {
                //mCurrentlyOnMission = false;
                // publishStop();
            }
        }
    }

    void patchOccupancyCallback(const ros_oadrive::StreetPatchOccupancy::ConstPtr &msg) {
        // Check if there is an obstruction, but only when backing up for a bypass
        // This prevents us from a lot of false obstructions when someone walks over streets
        // and also this helps to get a better view on the patch to be obstructed
        if (mCurrentState == StateMachine::State::BYPASS
            || mCurrentState == StateMachine::State::PRE_BYPASSING
            || mCurrentState == StateMachine::State::SWITCHING_FOR_BYPASS
            || mCurrentState == StateMachine::State::CURRENTLY_BYPASSING)
        {
            // We check if the left side of the patch is obstructed and we are trying to bypass..
            // In this case we cannot drive here
            if (msg->left_for_nav) {
                // Transform from local to world:
                geometry_msgs::PoseStamped localPose;
                localPose.header.frame_id = "local";
                localPose.pose.position.x = msg->x;
                localPose.pose.position.y = msg->y;
                localPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);


                geometry_msgs::PoseStamped worldPose;
                mTfListener.transformPose("world", ros::Time(0), localPose, "local", worldPose);
                
                // obstruction at worldPose.pose.position.x, worldPose.pose.position.y
                oadrive::core::ExtendedPose2d point(worldPose.pose.position.x, worldPose.pose.position.y, 0);
                if (!mNavigator.isObstructed(point)) {
                    ROS_INFO_STREAM("Obstruction while bypassing at " << worldPose.pose.position.x << "," << worldPose.pose.position.y << " reported!");

                    publishObstruction(point, true);

                    // if (!replan()) {
                    //     // publishStop();
                    // }
                }
            }
        }
    }

    void roadObstructionCallback(const geometry_msgs::PointStamped::ConstPtr &point_msg) {
        
        geometry_msgs::PoseStamped worldPose;
        geometry_msgs::PoseStamped sourcePose;
        sourcePose.header.frame_id = point_msg->header.frame_id;
        // sourcePose.header.stamp = point_msg->header.stamp;
        sourcePose.pose.position.x = point_msg->point.x;
        sourcePose.pose.position.y = point_msg->point.y;
        sourcePose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        mTfListener.transformPose("world", sourcePose, worldPose);
        
        // obstruction at worldPose.pose.position.x, worldPose.pose.position.y
        oadrive::core::ExtendedPose2d point(worldPose.pose.position.x, worldPose.pose.position.y, 0);

        // oadrive::core::ExtendedPose2d point(point_msg->point.x, point_msg->point.y, 0);

        // bool obstructed = mNavigator.toggleRoadObstruction(mTargetPose, mCurrentPose, point, mObstructedTiles);
        
        publishObstruction(point, !mNavigator.isObstructed(point));

        // if (!replan()) {
        //     mCurrentlyOnMission = false;
        //     // publishStop();
        // }
        return;
    }

    void patchCallback(const StreetPatchList::ConstPtr &patches_msg) {
        for (auto &patch : patches_msg->streetpatch)
        {
            if (patch.patch_type == "CROSS_SECTION") {
                unsigned int id = patch.ID;

                if (mSeenPatchIDs.find(id) == mSeenPatchIDs.end()) {
                    mSeenPatchIDs.insert(id);     
                    mNavigator.connectManeuver();
                }
            }
        }
    }

    bool replan() {
        
        if (mTeleOpMode) {
            return false;
        }

        // if (mManeuverList.size() == 0) {
        //     return true;
        // }

        mNavigator.saveCurrentBusStop();

        if (!generateNewPath()) {
            bool success = false;
            if (mBusMode) {
                int termCounter = 0;
                success = true;
                while(!generateBusPath()) {
                    termCounter++;
                    if (termCounter >= mNavigator.getNumberOfBusStops()) {
                        success = false;
                        break;
                    }
                }
            }

            if (!success) {
                ros_oadrive::Event msg;
                msg.type = "START_TELE_OP_EVENT";
                mPlanningEventPub.publish(msg);
            }

            return false;
        }

        if (!mManeuverList.empty()) {
            publishCurrentManeuver();
        }
        return true;
    }

    void timerCallback(const ros::TimerEvent& event) {
        replan();
    }

    void stateCallback(const StateChange::ConstPtr &msg)
    {
        auto prevState = static_cast<StateMachine::State>(msg->prevState);
        auto newState = static_cast<StateMachine::State>(msg->newState);
        mCurrentState = newState;

        if (prevState == StateMachine::State::ACTIVE || prevState == StateMachine::State::INACTIVE) {
            reset();
        }

        if (newState == StateMachine::TELE_OPERATION) {
            ROS_INFO_STREAM("ENTERING TELE OPERATION");

            mTeleOpMode = true;
        }

        // if teleop stops, reset and generate a new path to the target
        else if (prevState == StateMachine::TELE_OPERATION) {

            mTeleOpMode = false; 

            ROS_INFO_STREAM("LEAVING TELE OPERATION");

            reset();

            mNavigator.loadBusStop();

            if (!generateNewPath()) {

                if (mBusMode) {
                    int termCounter = 0;
                    while(!generateBusPath()) {
                        termCounter++;
                        if (termCounter >= mNavigator.getNumberOfBusStops()) {
                            return;
                        }
                    }
                }
            }

            publishGetReady();
            sleep(2);
            publishStart();
        }
    }

    void planningEventCallback(const ros_oadrive::Event::ConstPtr& event) {

        std::string type = event->type;

        if (type == "LEFT_EVENT_REGION") {

            // tell navigator that car has finished driving a cross section
            if (event->event_region.type == "CROSS_SECTION_REGION") {
                mNavigator.driveManeuver();
            }
        }
    }

    void trackedObjectsCallback(const TrackedObjects::ConstPtr &tracked_objects_msg) {

        if(!mPoseInitialized) {
            return;
        }

        if (!mTaxiMode || mCurrentlyOnMission) {
            return;
        }

        oadrive::obstacle::TrackedObjects trackedObjects = TrackedObjectsConverter::fromMessage(*tracked_objects_msg);

        bool personNearby = false;

        for (auto &person : trackedObjects.persons) {

            if (person.object.calcDistTo(mLocalPose) < 0.5) {
                personNearby = true;
            }
        }

        if (personNearby) {
            mCurrentlyOnMission = true;

            ROS_INFO_STREAM("Waiting for Person to get in...");
            ros_oadrive::Event msg;
            msg.type = "START_WAITING_EVENT";
            mPlanningEventPub.publish(msg);

            // generate random new target
            generateRandomNewPath();
        }
    }

    NavigatorNode(ros::NodeHandle nh)
        : mNavigator(Config::getString("Map", "Path", "")) {
        
        mManeuverPub = nh.advertise<ros_oadrive::JuryManeuver>("/aadc/jury/current_maneuver", 25);
        mEventPub = nh.advertise<std_msgs::String>("/aadc/jury/event", 25);
        mJuryPub = nh.advertise<std_msgs::String>("/aadc/jury/status", 25);
        mNavGoalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        mPubRvizPath = nh.advertise<visualization_msgs::Marker>("/aadc/visualization/path", 1);
        mPubRvizObstructions = nh.advertise<visualization_msgs::Marker>("/aadc/visualization/obstructions", 100);
        mPubRvizBusStops = nh.advertise<visualization_msgs::Marker>("/aadc/visualization/bus_stops", 100);
        mPlanningEventPub = nh.advertise<ros_oadrive::Event>("/aadc/planning/event", 1000);
        mNavObsPub = nh.advertise<ros_oadrive::NavObstruction>("/aadc/nav_obstructions", 1000);

        mEventSub = nh.subscribe("/aadc/jury/event", 25, &NavigatorNode::juryEventCallback, this);
        mPlanningEventSub = nh.subscribe("/aadc/planning/event", 100, &NavigatorNode::planningEventCallback, this);
        mCarPoseSub = nh.subscribe("/aadc/marker_position", 1, &NavigatorNode::carPoseCallback, this);
        mNavGoalSub = nh.subscribe("/move_base_simple/goal", 1, &NavigatorNode::navGoalCallback, this);
        mStateSub = nh.subscribe("/aadc/statemachine/state", 1, &NavigatorNode::stateCallback, this);
        mRoadObstructionSub = nh.subscribe("/clicked_point", 25, &NavigatorNode::roadObstructionCallback, this);
        mNavObsSub = nh.subscribe("/aadc/nav_obstructions", 25, &NavigatorNode::subObstruction, this);
        mPatchOccupancySub = nh.subscribe("/aadc/planning/patch_occupancy", 100, &NavigatorNode::patchOccupancyCallback, this);
        mTrackedObjectsSub = nh.subscribe("/aadc/object_tracking/tracked_objects", 1000,
                                          &NavigatorNode::trackedObjectsCallback, this);
        mPatchSub = nh.subscribe("/aadc/perception/patches", 10, &NavigatorNode::patchCallback, this);

        try {
            ROS_INFO("Waiting for transformations to be available..");
            mTfListener.waitForTransform("world", "local", ros::Time(0), ros::Duration(1000000.0));
            ROS_INFO("..Done!");
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        mReplanTimer = nh.createTimer(ros::Duration(1), &NavigatorNode::timerCallback, this, false);
        mReplanTimer.start();

        if (mBusMode) {
            mNavigator.initializeBusStop();
        }
    }

};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NavigatorNode");
  ros::NodeHandle nh("~");

  std::string config_folder;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  // Init stuff to get this going.
  oadrive::util::Config::setConfigPath(config_folder, car_name);

  ros_oadrive::NavigatorNode node(nh);

  // Wait for new subscription hits.
  ros::spin();

  return 0;
}