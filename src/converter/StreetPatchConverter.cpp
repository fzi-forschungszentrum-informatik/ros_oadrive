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
 * \author  Nico Kuhn <kuhn@fzi.de>
 * \date    2017
 * 
 * \author  Fabian Dürr
 * \date    2017
 *
 *
 */
//----------------------------------------------------------------------

#include "ros_oadrive/StreetPatchConverter.h"

#include <ros_oadrive/Pose2dConverter.h>

namespace ros_oadrive
{

StreetPatch StreetPatchConverter::toMessage(const oadrive::lanedetection::PatchHypothesis &obj)
{
  oadrive::lanedetection::PatchHypothesis::Type patch_type = obj.getPatchType();
  StreetPatch patch_msg;

  if (patch_type == oadrive::lanedetection::PatchHypothesis::Type::ROAD)
  {
    patch_msg.patch_type = "STRAIGHT";
  }
  else if (patch_type ==  oadrive::lanedetection::PatchHypothesis::Type::INTERSECTION)
  {
    patch_msg.patch_type = "CROSS_SECTION";
  }
  else if (patch_type ==  oadrive::lanedetection::PatchHypothesis::Type::PARKING)
  {
    patch_msg.patch_type = "PARKING";
  }
  else
  {
    throw std::invalid_argument("Unknown patch_type enum value.");
  }

  oadrive::lanedetection::OadrivePose patchPose = obj.getPose();

  patch_msg.ID = obj.getPatchID();

  if (obj.getDrivingDirection() == oadrive::lanedetection::PatchHypothesis::DrivingDirection::LEFT)
  {
    patch_msg.direction = "LEFT";
    patchPose.setYaw(patchPose.getYaw() - M_PI_2);
  }
  if (obj.getDrivingDirection() == oadrive::lanedetection::PatchHypothesis::DrivingDirection::STRAIGHT)
  {
    patch_msg.direction = "STRAIGHT";

  }
  if (obj.getDrivingDirection() == oadrive::lanedetection::PatchHypothesis::DrivingDirection::RIGHT)
  {
    patch_msg.direction = "RIGHT";
    patchPose.setYaw(patchPose.getYaw() + M_PI_2);
  }

  patch_msg.pose = Pose2dConverter::toMessage(patchPose);

  return patch_msg;
}

oadrive::world::PatchPtr StreetPatchConverter::fromMessage(const StreetPatch &msg)
{
  oadrive::world::PatchType patch_type;

  if (msg.patch_type == "STRAIGHT")
  {
    patch_type = oadrive::world::PatchType::STRAIGHT;
  }
  else if (msg.patch_type == "CROSS_SECTION")
  {
    patch_type = oadrive::world::PatchType::CROSS_SECTION;
  }
  else if (msg.patch_type == "PARKING")
  {
    patch_type = oadrive::world::PatchType::PARKING;
  }
  else
  {
    throw std::invalid_argument("Unknown patch_type: '" + msg.patch_type + "'");
  }


  oadrive::world::drivingDirection drivingDirection;
  if (msg.direction == "STRAIGHT")
  {
    drivingDirection = oadrive::world::drivingDirection::DD_STRAIGHT;
  }
  else if (msg.direction == "LEFT")
  {
    drivingDirection = oadrive::world::drivingDirection::DD_LEFT;
  }
  else if (msg.direction == "RIGHT")
  {
    drivingDirection = oadrive::world::drivingDirection::DD_RIGHT;
  }


  oadrive::core::ExtendedPose2d extended_pose(Pose2dConverter::fromMessage(msg.pose));
  oadrive::world::PatchPtr patch_obj(new oadrive::world::Patch(patch_type, extended_pose));
  patch_obj->setPatchID(msg.ID);
  patch_obj->setAction(drivingDirection);

  return patch_obj;
}

}  // namespace ros_oadrive
