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
 */
//----------------------------------------------------------------------

#include "ros_oadrive/EnvironmentObjectConverter.h"
#include "ros_oadrive/Pose2dConverter.h"

namespace ros_oadrive {

EnvironmentObject EnvironmentObjectConverter::toMessage(const oadrive::world::EnvObject &object) {
  EnvironmentObject object_msg;

  object_msg.id = object.getId();
  object_msg.width = object.getWidth();
  object_msg.length = object.getLength();
  object_msg.pose = Pose2dConverter::toMessage(object.getPose().getPose());

  return object_msg;
}

oadrive::world::EnvObject EnvironmentObjectConverter::fromMessage(
    const EnvironmentObject &object_msg) {
  oadrive::core::Pose2d pose = Pose2dConverter::fromMessage(object_msg.pose);
  oadrive::core::ExtendedPose2d extended_pose(pose);

  oadrive::world::EnvObject object(extended_pose, object_msg.width, object_msg.length);
  object.setId(object_msg.id);

  return object;
}

}  // namespace ros_oadrive
