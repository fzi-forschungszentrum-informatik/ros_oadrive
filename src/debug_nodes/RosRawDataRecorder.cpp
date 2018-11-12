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

#include <oadrive_core/ExtendedPose2d.h>

#include <boost/regex.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>

#include "ros_oadrive/StreetPatch.h"
#include <ros_oadrive/Pose2dConverter.h>
#include <ros_oadrive/MarkerPosition.h>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace oadrive::core;
using namespace boost;
using namespace sensor_msgs;
using namespace message_filters;

/**
 * This class can record an ros stream into simple files
 * There might be a use case with the oadrive test methods
 */

ExtendedPose2d lastPose;

bool recordingActive = true;

std::string recordFolder;
std::string carPoseFile;
std::string fileType = ".png";
std::stringstream ss;

int imagecount = 0;

std::string imagePrefix = "00000";
void callback(const sensor_msgs::ImageConstPtr& net, const sensor_msgs::ImageConstPtr& birdview, const ros_oadrive::MarkerPositionConstPtr& pos)
{
    try
    {
        cv_bridge::CvImagePtr birdviewPtr = cv_bridge::toCvCopy(birdview, "mono8");
        cv_bridge::CvImagePtr netPtr = cv_bridge::toCvCopy(net, "mono8");

        bool animate = true;

        cv::Mat birdviewImg = birdviewPtr->image;
        cv::Mat netImg = netPtr->image;

        if (!birdviewImg.data || !netImg.data)
        {
            cv::waitKey(0);
            return;
        } 
     
        if (recordingActive) {
            //Write Birdview
            ss << recordFolder << "/" << imagePrefix << boost::lexical_cast<std::string>(imagecount) << fileType;
            std::string fullPath = ss.str();
            ss.str("");
            imwrite(fullPath, birdviewImg);

            // Write net
            ss << recordFolder << "/net" << imagePrefix << boost::lexical_cast<std::string>(imagecount) << fileType;
            fullPath = ss.str();
            ss.str("");
            imwrite(fullPath, netImg);

            // always keep the current prefix, used for 1 -> 00001
            if (imagecount == 9 || imagecount == 99 || imagecount == 999 || imagecount == 9999 || imagecount == 99999) {
                imagePrefix.erase(0,1);
            }
            imagecount++;

            std::ofstream myfile(carPoseFile.c_str(), std::ios::binary | std::ios::out | std::ios::app);
            if (myfile) {
                const oadrive::core::Pose2d pose = ros_oadrive::Pose2dConverter::fromMessage(pos->pose);
                oadrive::core::ExtendedPose2d extendedPose(pose);

                double x = extendedPose.getX();
                double y = extendedPose.getY();
                double yaw = extendedPose.getYaw();

                // Write in format for test_oadrive_lanedet
                myfile << x << ":" << y << ":" << yaw << std::endl;

                /*
                myfile.write( reinterpret_cast<char*>( &x ), sizeof x );
                myfile.write( reinterpret_cast<char*>( &y ), sizeof y );
                myfile.write( reinterpret_cast<char*>( &yaw ), sizeof yaw );
                */
                // Close the file to unlock it
                myfile.close();
            }
        }
      
        // imshow( "output", birdviewImg );
        // int code = code = cv::waitKey(animate ? 1 : 0);
        //   if (code == 27)   // ESC to abort
        //     //return;
        //   if (code == 114)   // r to toggle record
        //     recordingActive = !recordingActive;
        //   else if ( code == 65 || code == 97 )
        //     animate = !animate;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("ERROR: Perception_node_ros : imageCallback()");
    }
}

void poseCallback(const ros_oadrive::MarkerPosition::ConstPtr& msg) {
    const oadrive::core::Pose2d pose = ros_oadrive::Pose2dConverter::fromMessage(msg->pose);
    oadrive::core::ExtendedPose2d extended_pose(pose);

    lastPose = extended_pose;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RosRawDataRecorder");
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
    carPoseFile = recordFolder + "/carPose.txt";
    std::string fileCreateCommand = "touch " + carPoseFile + " || exit ";
    std::system(fileCreateCommand.c_str());

    // Subscribe
    message_filters::Subscriber<Image> net(nh, "/aadc/net", 1);
    message_filters::Subscriber<Image> birdview(nh, "/aadc/birdview", 2);
    message_filters::Subscriber<ros_oadrive::MarkerPosition> pose(nh, "/aadc/marker_position", 4);
  

    typedef sync_policies::ApproximateTime<Image, Image, ros_oadrive::MarkerPosition> ApproximateTime;
    Synchronizer<ApproximateTime> sync(ApproximateTime(10), net, birdview, pose);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  
    ros::spin();
}
