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

#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/ExtendedPose2d.h>

#include <boost/regex.hpp>

#include <ros/ros.h>

#include <ros_oadrive/Pose2DStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_srvs/Empty.h>

#include <oadrive_world/MultiTrajectory.h>

#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace boost;
using namespace message_filters;

/**
 * This class can record an ros stream into simple files
 * There might be a use case with the oadrive test methods
 */

bool recordingActive = false;
bool drivingForward = true;
bool isInitialized = false;

std::string recordFolder;
std::string trajectoryFile;
std::string fileType = ".png";
std::stringstream ss;

int imagecount = 0;

double xOffset;
double yOffset;
double yawOffset; 

std::string imagePrefix = "00000";

oadrive::world::MultiTrajectory traj;

void reset() {
    traj.trajectories.clear();
    isInitialized = false;
}

void startNewSubTrajectory() {
    ROS_INFO_STREAM("Starting new sub trajectory.");
    oadrive::core::Trajectory2d newTraj;
    newTraj.isForwardTrajectory() = drivingForward;
    traj.trajectories.push_back(newTraj);
}

void addPoint(double x, double y, double yaw) {
    oadrive::core::ExtendedPose2d pose(x, y, yaw);
    traj.trajectories.back().push_back(pose);
}

void callback(const ros_oadrive::Pose2DStamped::ConstPtr& pos, const ackermann_msgs::AckermannDriveStamped::ConstPtr& drive)
{
    if (recordingActive) {

        float speed = (drive->drive).speed;

        double x = pos->x;
        double y = pos->y;
        double yaw = pos->theta;

        if (!isInitialized) {
            xOffset = x;
            yOffset = y;
            yawOffset = yaw;
            drivingForward = speed < 0.0f ? false : true;
            startNewSubTrajectory();
            isInitialized = true;
        }

        if (drivingForward && speed < 0.0f) {
            drivingForward = false;
            startNewSubTrajectory();
        }
        else if (!drivingForward && speed > 0.0f) {
            drivingForward = true;
            startNewSubTrajectory();
        }        

        addPoint(x-xOffset, y-yOffset, yaw-yawOffset);

        /*std::ofstream myfile(trajectoryFile.c_str(), std::ios::binary | std::ios::out | std::ios::app);
        if (myfile) {
            

            // Write in format for test_oadrive_lanedet
            //myfile << x << ":" << y << ":" << yaw << std::endl;

            myfile.close();
        }*/
    }

    else {
        
    }
}

void writeToFile() {
    int idx = 0;
    for (auto const& subTraj : traj.trajectories) {
        std::stringstream fileName;

        fileName << trajectoryFile << "_" << idx << ".txt";
        std::ofstream myfile(fileName.str().c_str(), std::ios::out);
        
        if (myfile) {
            myfile << subTraj;
            myfile.close();
        }

        idx++;
    }
}

bool cbToggle(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    recordingActive = !recordingActive;
    if (recordingActive) 
        ROS_INFO_STREAM("START RECORDING");
    else
        ROS_INFO_STREAM("STOP RECORDING");

    if (isInitialized) {
            ROS_INFO_STREAM("FINISHED. Lenght: " << traj.trajectories.size());
            //ROS_INFO_STREAM(traj.trajectories.back());
            writeToFile();
            reset();
    }

    return recordingActive;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RosTrajectoryRecorder");
    ros::NodeHandle nh("~");
  
   // read ROS params
    nh.param<std::string>("folder", recordFolder, "");

    if (!nh.hasParam("folder")) {
        ROS_ERROR("No recording folder given. (_folder:=/home/.../)");
        return 0;
    }

    nh.getParam("folder", recordFolder);
    ROS_INFO_STREAM("Recording into \"" << recordFolder << "\" ...");

    std::string folderCreateCommand = "mkdir -p " + recordFolder;
    std::system(folderCreateCommand.c_str());


    //Write Car Pose to Binary File
    trajectoryFile = recordFolder + "/trajectory";
    std::string fileCreateCommand = "touch " + trajectoryFile + " || exit ";
    std::system(fileCreateCommand.c_str());

    // Subscribe
    //ros::Subscriber sub = nh.subscribe("/aadc/pose_2d", 4, &callback);

    // advertise
    ros::ServiceServer service = nh.advertiseService("/aadc/trajectory_recorder/toggle", &cbToggle);

    message_filters::Subscriber<ros_oadrive::Pose2DStamped> pose(nh, "/aadc/pose_2d", 4);
    message_filters::Subscriber<ackermann_msgs::AckermannDriveStamped> drive(nh, "/aadc/control/car_stamped", 4);

    typedef sync_policies::ApproximateTime<ros_oadrive::Pose2DStamped, ackermann_msgs::AckermannDriveStamped> ApproximateTime;
    Synchronizer<ApproximateTime> sync(ApproximateTime(10), pose, drive);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
}
