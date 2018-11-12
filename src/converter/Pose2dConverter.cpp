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
 *
 * \author  Kolja Esders <esders@fzi.de>
 * \date    2017
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 */
//----------------------------------------------------------------------

#include "ros_oadrive/Pose2dConverter.h"

using namespace geometry_msgs;

namespace ros_oadrive
{

Pose2D Pose2dConverter::toMessage(const oadrive::core::Pose2d &obj)
{
  Pose2D pose_msg;
  pose_msg.x = obj.translation()[0];
  pose_msg.y = obj.translation()[1];
  pose_msg.theta = oadrive::core::PoseTraits<oadrive::core::Pose2d>::yaw(obj);

  return pose_msg;
}

Pose2D Pose2dConverter::toMessage(const oadrive::lanedetection::OadrivePose &obj)
{
  Pose2D pose_msg;
  pose_msg.x = obj.getX();
  pose_msg.y = obj.getY();
  pose_msg.theta = obj.getYaw();

  return pose_msg;
}

oadrive::core::Pose2d Pose2dConverter::fromMessage(const Pose2D &msg)
{
  oadrive::core::Pose2d pose_obj;
  pose_obj.translation()[0] = msg.x;
  pose_obj.translation()[1] = msg.y;
  oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromOrientationRPY(
          pose_obj, msg.theta);

  return pose_obj;
}

oadrive::lanedetection::OadrivePose
Pose2dConverter::fromMessageToOadrivePose(const Pose2D &msg)
{
  oadrive::lanedetection::OadrivePose pose;
  pose.setX(msg.x);
  pose.setY(msg.y);
  pose.setYaw(msg.theta);

  return pose;
}

}  // namespace ros_oadrive

