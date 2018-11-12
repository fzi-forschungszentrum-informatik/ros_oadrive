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
 * \author  Fabian Dürr
 * \date    2017
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#ifndef ROS_OADRIVE_ROAD_PERCEPTION_NOAD_H_
#define ROS_OADRIVE_ROAD_PERCEPTION_NOAD_H_

#include <oadrive_lanedetection/RoadPatching/RoadPerception.h>
#include <oadrive_core/ExtendedPose2d.h>
#include <ros_oadrive/ImagePosition.h>
#include <ros_oadrive/Event.h>
#include <ros_oadrive/StateChange.h>
#include <oadrive_world/StateMachine.h>
#include <ros_oadrive/JuryManeuver.h>

#include <std_msgs/String.h>


namespace ros_oadrive
{

class RoadPerceptionNode {

 public:
  /*!
    \brief Handles initial publish & subscribe.
   */
  RoadPerceptionNode(ros::NodeHandle nodeHandle,
                     oadrive::util::CoordinateConverter* coordConv,
                     oadrive::lanedetection::RoadPerception* perception);

  /*!becaue
   * Stores the received position received from the odometry. This position is more
   * inaccurate because it is never synchronized and as a result will drift away from the actual
   * position over the time.
   * @param msg contains the drifting position.
   */
  void driftingPoseCallback(const geometry_msgs::Pose2D::ConstPtr &pose_msg);

  /*!
   * Stores the received position received from the odometry. This position should be more
   * accurate the the drifting position becaue it is synchronized via landmarks (e.g. traffic signs)
   * @param msg contains the synchronized position.
   */
  void syncPoseCallback(const geometry_msgs::Pose2D::ConstPtr &msg);

  /*!
   * Stores the received birdview image.
   * @param msg contains the birdview image.
   */
  void birdviewImageCallback(const ros_oadrive::ImagePosition::ConstPtr& msg);

  /*!
  * Stores the received neural net image which should be a segmentation of the birdview image in
  * road, interesection or neither.
  * @param msg contains the neural net image.
  */
  void neuralNetCallback(const sensor_msgs::ImageConstPtr &msg);

   /*!
  * Changes the mode of the road perception according to the current state.
  * @param msg contains the last state transition.
  */
  void stateCallback(const ros_oadrive::StateChange::ConstPtr &msg);

  void trafficSignCallback(const TrackedObjects::ConstPtr &trackedObjects);

  void maneuverCommandCallback(const ros_oadrive::JuryManeuver::ConstPtr& msg);

  void eventCallback(const Event::ConstPtr &event_msg);

  void mapCallback(const std_msgs::String::ConstPtr &msg);

  void run();

 private:

  void publishRoad(const oadrive::lanedetection::PatchHypothesisList& road);

  void addManeuverCommand(const std::string maneuverCommand);

  /*!
    \brief Advertise topics that will be published.
   */
  void advertise();

  /*!
    \brief Subscribe to necessary topics.
   */
  void subscribe();

  void teleOpTimer(const ros::TimerEvent& event);

  void showDebugImage();

  oadrive::lanedetection::RoadPerception* mPerception = NULL;
  oadrive::util::CoordinateConverter* mCoordinateConverter = NULL;

  oadrive::lanedetection::OadrivePose mCurrentPose;
  oadrive::lanedetection::OadrivePose mCurrentPoseAccurate;
  std_msgs::Header mCurrentHeader;

  oadrive::world::TrafficSign mCurrentTrafficSign;

  cv::Mat mCurrentBirdviewRGB;
  cv::Mat mCurrentBirdviewGray;
  cv::Mat mCurrentNeuralNetOutput;

  bool mNewBirdviewRecieved;
  bool mNewNNOutputRecieved;
  bool mTrafficSignReceived;

  bool mFirstManeuverReceived;

  ros::NodeHandle mNode;

  image_transport::ImageTransport mImageTransport;

  ros::Subscriber mImageSubscriber;
  image_transport::Subscriber mNeuralNetSubscriber;
  ros::Subscriber mTrafficSignSubscriber;
  ros::Subscriber mManeuverCommandSubscriber;
  ros::Subscriber mStateSubscriber;
  ros::Subscriber mMapSubscriber;

  ros::Timer mTeleOpTimer;

  ros::Publisher mPatchPublisher;
  image_transport::Publisher mDebugPublisher;
};

}  // namespace ros_oadrive

#endif /* ROS_OADRIVE_ROAD_PERCEPTION_NOAD_H_ */
