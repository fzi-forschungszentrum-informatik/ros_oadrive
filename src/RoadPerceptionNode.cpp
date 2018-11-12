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
 * \author  Nico Kuhn <kuhn@fzi.de>
 * \date    2017
 * 
 * \author  Fabian Dürr
 * \date    2017
 * 
 * \author  Kai Braun <kbraun@fzi.de>
 * \date    2017
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include <ros_oadrive/StreetPatch.h>
#include <ros_oadrive/StreetPatchConverter.h>
#include <ros_oadrive/Pose2dConverter.h>
#include <ros_oadrive/StreetPatchList.h>

#include <iostream>
#include <fstream>

#include <oadrive_util/Config.h>
#include <boost/regex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <ros_oadrive/TrafficSignConverter.h>
#include <std_msgs/String.h>
#include <oadrive_obstacle/ObjectTracker.h>
#include <ros_oadrive/TrackedObjects.h>
#include <ros_oadrive/TrackedObjectsConverter.h>
#include <ros_oadrive/Event.h>

#include "ros_oadrive/RoadPerceptionNode.h"


using namespace oadrive::core;
using namespace oadrive::lanedetection;
using namespace oadrive::util;
using namespace boost;

namespace ros_oadrive
{

int counter = 0;
std::ofstream myfile;
bool init = true;

RoadPerceptionNode::RoadPerceptionNode(ros::NodeHandle nodeHandle, CoordinateConverter* coordConv,
                                       RoadPerception* perception)
        : mNode(nodeHandle),
          mImageTransport(nodeHandle),
          mPerception(perception),
          mCoordinateConverter(coordConv),
          mNewBirdviewRecieved(false),
          mNewNNOutputRecieved(false),
          mTrafficSignReceived(false),
          mFirstManeuverReceived(false)


{
  advertise();
  subscribe();

  mTeleOpTimer = mNode.createTimer(ros::Duration(1), &RoadPerceptionNode::teleOpTimer, this, false);
  mTeleOpTimer.stop();

  mPerception->configureRoadPerception(true);
  ExtendedPose2d localVotePose(1.0, 0.25, 0.0);
  mPerception->setEstimatedFirstPatchPose(localVotePose);
}

void RoadPerceptionNode::run()
{
  tf::TransformListener listener;

  try {
    ROS_INFO("Waiting for transformations to be available..");
    listener.waitForTransform("world", "local", ros::Time(0), ros::Duration(10.0));
    ROS_INFO("..Done!");
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }

  while (ros::ok())
  {
    std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();
    // wait until a new birdview image and position were received
    while (!mNewBirdviewRecieved && ros::ok())
    {
      ros::spinOnce();
      // wake up every ms to check for new messages
      ros::Duration(0.001).sleep();

    }

    mNewBirdviewRecieved = false;
    if (!mFirstManeuverReceived) continue;

    std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    #ifdef DEBUG_ACTIVE
    ROS_INFO_STREAM("Waited for birdview " << elapsed_seconds);
    #endif
    // ---------------------------------------------------------------------------------

    // Execute the feature detection while waiting on the neural net output
    mPerception->updateCarPose(mCurrentPose);
    mPerception->executeFeatureDetection(mCurrentBirdviewGray);

    std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
    elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    // ROS_INFO_STREAM("Feature detection " << elapsed_seconds);
    // ---------------------------------------------------------------------------------

    // if not already received wait for the neural net output
    // while (!mNewNNOutputRecieved && ros::ok())
    // {
    //   ros::spinOnce();
    //   if (!mFirstManeuverReceived) break;
    //   // wake up every ms to check for new messages
    //   ros::Duration(0.001).sleep();
    // }
    // mNewNNOutputRecieved = false;

    if (!mFirstManeuverReceived) continue;

    std::chrono::time_point<std::chrono::system_clock> t3 = std::chrono::system_clock::now();
    elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
    // ROS_INFO_STREAM("Waiting for net " << elapsed_seconds);
    // ---------------------------------------------------------------------------------

    OadrivePose oaWorldPose;
    try {
      geometry_msgs::PoseStamped localPose;
      localPose.header.frame_id = "local";
      localPose.header.stamp = mCurrentHeader.stamp;
      localPose.pose.position.x = mCurrentPose.getX();
      localPose.pose.position.y = mCurrentPose.getY();
      localPose.pose.orientation = tf::createQuaternionMsgFromYaw(mCurrentPose.getYaw());

      geometry_msgs::PoseStamped worldPose;
      listener.transformPose("world", ros::Time(0), localPose, "local", worldPose);
      // listener.waitForTransform("world", "local", mCurrentHeader.stamp, ros::Duration(0.05));
      // listener.transformPose("world", localPose, worldPose);

      tf::Quaternion q(
        worldPose.pose.orientation.x,
        worldPose.pose.orientation.y,
        worldPose.pose.orientation.z,
        worldPose.pose.orientation.w
      );
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      oaWorldPose = OadrivePose(worldPose.pose.position.x, worldPose.pose.position.y, yaw);
      // mPerception->executeRoadPerception(mCurrentNeuralNetOutput, &mCurrentTrafficSign);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }

    mPerception->executeRoadPerception(oaWorldPose);
    // Execute the actual road perception
    // if (mTrafficSignReceived)
    // {
    //   mPerception->executeRoadPerception(mCurrentNeuralNetOutput, &mCurrentTrafficSign);
    //   mTrafficSignReceived = false;
    // }
    // else
    // {
    //   mPerception->executeRoadPerception(mCurrentNeuralNetOutput, nullptr);
    // }
    publishRoad(mPerception->getRoad());
    std::chrono::time_point<std::chrono::system_clock> t4 = std::chrono::system_clock::now();
    elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
    // ROS_INFO_STREAM("road perception " << elapsed_seconds);
    // ---------------------------------------------------------------------------------

    showDebugImage();

    std::chrono::time_point<std::chrono::system_clock> t5 = std::chrono::system_clock::now();
    elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();
    // ROS_INFO_STREAM("debug " << elapsed_seconds);
    // ---------------------------------------------------------------------------------
  }
}

void RoadPerceptionNode::neuralNetCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    mCurrentNeuralNetOutput = cv_ptr->image;

    if (!mCurrentNeuralNetOutput.data)
    {
      return;
    }
    mNewNNOutputRecieved = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("ERROR: Perception_node_ros : imageCallback()");
  }
}

void RoadPerceptionNode::birdviewImageCallback(const ImagePosition::ConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg->image, "mono8");
    mCurrentBirdviewGray = cv_ptr->image;
    mCurrentPose = ros_oadrive::Pose2dConverter::fromMessageToOadrivePose(msg->pose);
    mCurrentHeader = msg->image.header;

    if (!mCurrentBirdviewGray.data)
    {
      cv::waitKey(0);
      return;
    }

    mNewBirdviewRecieved = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("ERROR: Perception_node_ros : imageCallback()");
  }
}

void RoadPerceptionNode::trafficSignCallback(const TrackedObjects::ConstPtr &trackedObjects)
{
	oadrive::obstacle::TrackedObjects oadrivetrackedObjects = TrackedObjectsConverter::fromMessage(*trackedObjects);
  if( oadrivetrackedObjects.trafficSigns.size() > 0)
  {
	  mCurrentTrafficSign = oadrivetrackedObjects.trafficSigns.front();
	  mTrafficSignReceived = true;
  }
}

void RoadPerceptionNode::maneuverCommandCallback(const ros_oadrive::JuryManeuver::ConstPtr &msg)
{
  if (!mFirstManeuverReceived)
  {
    mFirstManeuverReceived = true;
  }

  addManeuverCommand(msg->action);
}

void RoadPerceptionNode::stateCallback(const ros_oadrive::StateChange::ConstPtr &msg) 
{
  auto prevState = static_cast<StateMachine::State>(msg->prevState);
  auto newState = static_cast<StateMachine::State>(msg->newState);

  if (prevState == StateMachine::State::ACTIVE || prevState == StateMachine::State::INACTIVE) {
    mFirstManeuverReceived = false;
    mPerception->reset(OadrivePose(1.0f, 0.25f, 0.f));
    mNewBirdviewRecieved = false;
    mNewNNOutputRecieved = false;
    mTrafficSignReceived = false;
    mPerception->setMergeMode(false);
    mTeleOpTimer.stop();
  }

  // handle entered states
  switch(newState) {
    case StateMachine::TELE_OPERATION:
      mTeleOpTimer.start();
      break;
    case StateMachine::PULLING_OUT_LEFT:
      mPerception->reset(OadrivePose(1.1f, 0.f, M_PI_2));
      break;
    case StateMachine::PULLING_OUT_RIGHT:
      mPerception->reset(OadrivePose(1.1f, 0.f, -M_PI_2));
      break;
    case StateMachine::PRE_MERGING:
      // we are coming from the ramp, where we drive a fixed trajectory, so we have to reset our road perception
      ROS_INFO_STREAM("RESET ROAD PERCEPTION AFTER RAMP");
      // mPerception->reset(OadrivePose(0.6f, 0.25f, M_PI / 8)); // Used at qualifying
      // mPerception->reset(OadrivePose(0.6f, 0.f, M_PI / 8));

      // Works better at home
      mPerception->reset(OadrivePose(0.7f, 0.22f, M_PI / 8));


      ROS_INFO_STREAM("ACTIVATING MERGE MODE!");
      mPerception->setMergeMode(true);
      break;
    default:
      break;
  }

  // handle left states
  switch(prevState) {
    case StateMachine::TELE_OPERATION:
      mTeleOpTimer.stop();
      break;
    case StateMachine::MERGING:
      ROS_INFO_STREAM("DEACTIVATING MERGE MODE!");
      mPerception->setMergeMode(false);
      break;
    default:
      break;
  }
}

void RoadPerceptionNode::mapCallback(const std_msgs::String::ConstPtr &msg) {
  mPerception->loadMap(msg->data);
}


void RoadPerceptionNode::addManeuverCommand(const std::string maneuverCommand)
{
  if (maneuverCommand ==  "left")
  {
    mPerception->addNextDrivingDirection(PatchHypothesis::DrivingDirection::LEFT);
  }
  else if (maneuverCommand ==  "right")
  {
    mPerception->addNextDrivingDirection(PatchHypothesis::DrivingDirection::RIGHT);
  }
  else if (maneuverCommand ==  "straight")
  {
    mPerception->addNextDrivingDirection(PatchHypothesis::DrivingDirection::STRAIGHT);
  }
}

void RoadPerceptionNode::publishRoad(const oadrive::lanedetection::PatchHypothesisList &road)
{
  //Transform patches and publish them
  std::vector<StreetPatch> patchMessages;
  for (const PatchHypothesis &patchHyp : road)
  {
    PatchHypothesis patch = patchHyp;
    patch.setPose(mCoordinateConverter->car2World(mCurrentPose, patch.getPose()));
    patchMessages.push_back(StreetPatchConverter::toMessage(patch));
  }

  StreetPatchList patchListMessage;
  patchListMessage.streetpatch = patchMessages;
  mPatchPublisher.publish(patchListMessage);
}

void RoadPerceptionNode::advertise()
{
  mPatchPublisher = mNode.advertise<StreetPatchList>("/aadc/perception/patches", 1);
  mDebugPublisher = mImageTransport.advertise("/aadc/perception/debug", 1);
}

void RoadPerceptionNode::subscribe()
{
  mImageSubscriber = mNode.subscribe("/aadc/front/birdview", 1,
                                     &RoadPerceptionNode::birdviewImageCallback, this);
  mNeuralNetSubscriber = mImageTransport.subscribe("/aadc/net", 1,
                                                   &RoadPerceptionNode::neuralNetCallback, this);
  mTrafficSignSubscriber = mNode.subscribe("/aadc/object_tracking/tracked_objects", 1,
                                           &RoadPerceptionNode::trafficSignCallback, this);
  mManeuverCommandSubscriber = mNode.subscribe("/aadc/jury/current_maneuver", 1,
                                               &RoadPerceptionNode::maneuverCommandCallback, this);
  mStateSubscriber = mNode.subscribe("/aadc/statemachine/state", 10, &RoadPerceptionNode::stateCallback, this);

  mMapSubscriber = mNode.subscribe("/aadc/alpaka_map", 1000, &RoadPerceptionNode::mapCallback, this); 
                                  
}

void RoadPerceptionNode::teleOpTimer(const ros::TimerEvent& event) {
  mPerception->reset(OadrivePose(1.0f, 0.25f, 0.f));
}

void RoadPerceptionNode::showDebugImage()
{
  // save some cpu cycles if there is no one listening
  if (mDebugPublisher.getNumSubscribers() > 0) {
    // Show birdview with features and patches
    cv::Mat features = mPerception->generateDebugImage();

    cv::Mat channels[4];
    cv::split(features, channels);

    cv::Mat birdViewColor;
    cv::cvtColor(mCurrentBirdviewGray, birdViewColor, CV_GRAY2BGR);

    cv::Mat sum;
    cv::bitwise_or(birdViewColor, 0, sum, 255 - channels[3]);
    cv::Mat featuresRGB;

    cv::cvtColor(features, featuresRGB, CV_BGRA2BGR);
    sum = sum + featuresRGB;

    // cv::imshow("output", sum);
    // cv::imshow("outputNet", mCurrentNeuralNetOutput);
    // cv::waitKey(1);

    std_msgs::Header header;
    header.stamp = ros::Time::now();

    sensor_msgs::ImageConstPtr debugImgMsg = cv_bridge::CvImage(header, "bgr8", sum).toImageMsg();
    mDebugPublisher.publish(debugImgMsg);
  }
}

} //ros_oadrive

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RoadPerceptionNode");
  ros::NodeHandle nodeHandle("~");

  std::string config_folder;
  std::string car_name;
  nodeHandle.param<std::string>("config_folder", config_folder, "");
  nodeHandle.param<std::string>("car_name", car_name, "");

  std::string mConfigPath(Config::setConfigPath(config_folder, car_name));
  std::string mBirdViewCalFile(
          (boost::filesystem::path(config_folder) / car_name / "BirdviewCal.yml").string());

  CoordinateConverter* coordinateConverter = new CoordinateConverter(mBirdViewCalFile);

  RoadPerception* perception = new RoadPerception(coordinateConverter->getImgSizeBirdView(),
                                                  coordinateConverter, true);

  ros_oadrive::RoadPerceptionNode perceptionNode(nodeHandle, coordinateConverter, perception);
  perceptionNode.run();
}

