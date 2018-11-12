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
 */
//----------------------------------------------------------------------

#ifndef ROS_OADRIVE_POSE_2D_CONVERTER_H_
#define ROS_OADRIVE_POSE_2D_CONVERTER_H_

#include <geometry_msgs/Pose2D.h>
#include <oadrive_core/Pose.h>
#include <oadrive_lanedetection/FeatureDetection/StreetTypes.h>

namespace ros_oadrive {

class Pose2dConverter {
 public:
  static geometry_msgs::Pose2D toMessage(const oadrive::core::Pose2d &pose);

  static geometry_msgs::Pose2D toMessage(const oadrive::lanedetection::OadrivePose &pose);

  static oadrive::core::Pose2d fromMessage(const geometry_msgs::Pose2D &pose);

  static oadrive::lanedetection::OadrivePose fromMessageToOadrivePose(const geometry_msgs::Pose2D
                                                                   &msg);
};

}  // namespace ros_oadrive

#endif /* ROS_OADRIVE_POSE_2D_CONVERTER_H_ */

