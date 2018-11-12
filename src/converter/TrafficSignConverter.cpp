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

#include "ros_oadrive/TrafficSignConverter.h"
#include "ros_oadrive/EnvironmentObjectConverter.h"

namespace ros_oadrive {

TrafficSign TrafficSignConverter::toMessage(const oadrive::world::TrafficSign &traffic_sign) {
  TrafficSign traffic_sign_msg;

  EnvironmentObject env_object_msg = EnvironmentObjectConverter::toMessage(traffic_sign);
  traffic_sign_msg.object = env_object_msg;
  std::string type = "";

  if (traffic_sign.getType() == oadrive::world::UNMARKED_INTERSECTION) {
    type = "UNMARKED_INTERSECTION";
  } else if (traffic_sign.getType() == oadrive::world::STOP_AND_GIVE_WAY) {
    type = "STOP_AND_GIVE_WAY";
  } else if (traffic_sign.getType() == oadrive::world::PARKING_AREA) {
    type = "PARKING_AREA";
  } else if (traffic_sign.getType() == oadrive::world::HAVE_WAY) {
    type = "HAVE_WAY";
  } else if (traffic_sign.getType() == oadrive::world::GIVE_WAY) {
    type = "GIVE_WAY";
  } else if (traffic_sign.getType() == oadrive::world::PEDESTRIAN_CROSSING) {
    type = "PEDESTRIAN_CROSSING";
  } else if (traffic_sign.getType() == oadrive::world::TEST_COURSE_A9) {
    type = "TEST_COURSE_A9";
  } else if (traffic_sign.getType() == oadrive::world::ROAD_WORKS) {
    type = "ROAD_WORKS";
  }
  traffic_sign_msg.type = type;

  return traffic_sign_msg;
}

oadrive::world::TrafficSign TrafficSignConverter::fromMessage(const TrafficSign &traffic_sign_msg) {
  oadrive::world::TrafficSignType type;
  if (traffic_sign_msg.type == "UNMARKED_INTERSECTION") {
    type = oadrive::world::UNMARKED_INTERSECTION;
  } else if (traffic_sign_msg.type == "STOP_AND_GIVE_WAY") {
    type = oadrive::world::STOP_AND_GIVE_WAY;
  } else if (traffic_sign_msg.type == "PARKING_AREA") {
    type = oadrive::world::PARKING_AREA;
  } else if (traffic_sign_msg.type == "HAVE_WAY") {
    type = oadrive::world::HAVE_WAY;
  } else if (traffic_sign_msg.type == "GIVE_WAY") {
    type = oadrive::world::GIVE_WAY;
  } else if (traffic_sign_msg.type == "PEDESTRIAN_CROSSING") {
    type = oadrive::world::PEDESTRIAN_CROSSING;
  } else if (traffic_sign_msg.type == "TEST_COURSE_A9") {
    type = oadrive::world::TEST_COURSE_A9;
  } else if (traffic_sign_msg.type == "ROAD_WORKS") {
    type = oadrive::world::ROAD_WORKS;
  }

  oadrive::world::EnvObject env_object =
      EnvironmentObjectConverter::fromMessage(traffic_sign_msg.object);
  oadrive::world::TrafficSign traffic_sign(type, env_object.getPose());

  return traffic_sign;
}

oadrive::world::TrafficSignType TrafficSignConverter::getTypeFromId(int id) {
  if (id == 0) {
    return oadrive::world::UNMARKED_INTERSECTION;
  } else if (id == 1) {
    return oadrive::world::STOP_AND_GIVE_WAY;
  } else if (id == 2) {
    return oadrive::world::PARKING_AREA;
  } else if (id == 3) {
    return oadrive::world::HAVE_WAY;
  } else if (id == 5) {
    return oadrive::world::GIVE_WAY;
  } else if (id == 6) {
    return oadrive::world::PEDESTRIAN_CROSSING;
  } else if (id == 10) {
    return oadrive::world::TEST_COURSE_A9;
  } else if (id == 12) {
    return oadrive::world::ROAD_WORKS;
  } 

  return oadrive::world::INVALID;
}

}  // namespace ros_oadrive
