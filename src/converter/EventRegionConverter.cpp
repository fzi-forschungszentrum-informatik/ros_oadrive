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
 * 
 */
//----------------------------------------------------------------------

#include "ros_oadrive/EventRegionConverter.h"
#include "ros_oadrive/Pose2dConverter.h"

namespace ros_oadrive {

EventRegion EventRegionConverter::toMessage(
    const oadrive::world::EventRegion &obj) {
  EventRegion msg;

  oadrive::world::EventRegionType type = obj.getEventRegionType();
  if (type == oadrive::world::CROSS_SECTION_REGION) {
    msg.type = "CROSS_SECTION_REGION";
  }
  else if (type == oadrive::world::CROSS_SECTION_HALT) {
    msg.type = "CROSS_SECTION_HALT";
  }
  else if (type == oadrive::world::PARKING_SPACE_REGION) {
    msg.type = "PARKING_SPACE_REGION";
  } else {
    throw std::invalid_argument("Unknown event region type enum value.");
  }

  msg.uid = obj.getId();
  msg.pose = Pose2dConverter::toMessage(obj.getLocalPose());
  msg.parking_take_off = Pose2dConverter::toMessage(obj.getParkingTakeoff());
  msg.width = obj.getWidth();
  msg.length = obj.getLength();
  msg.one_time = obj.getOneTime();

  return msg;
}

oadrive::world::EventRegion EventRegionConverter::fromMessage(
    const EventRegion &msg) {
  oadrive::world::EventRegionType type;

  if (msg.type == "CROSS_SECTION_REGION") {
    type = oadrive::world::CROSS_SECTION_REGION;
  } else if (msg.type == "CROSS_SECTION_HALT") {
    type = oadrive::world::CROSS_SECTION_HALT;
  } else if (msg.type == "PARKING_SPACE_REGION") {
    type = oadrive::world::PARKING_SPACE_REGION;
  } else {
    throw std::invalid_argument("Unknown event region type: '" + msg.type + "'");
  }

  oadrive::core::Pose2d pose = Pose2dConverter::fromMessage(msg.pose);
  oadrive::core::Pose2d parkingTakeoff = Pose2dConverter::fromMessage(msg.parking_take_off);
  oadrive::world::EventRegion obj(type, pose, parkingTakeoff, msg.width, msg.length, msg.one_time);
  obj.setId(msg.uid);

  return obj;
}

}  // namespace ros_oadrive

