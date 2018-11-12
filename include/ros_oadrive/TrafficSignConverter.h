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

#ifndef ROS_OADRIVE_TRAFFIC_SIGN_CONVERTER_H_
#define ROS_OADRIVE_TRAFFIC_SIGN_CONVERTER_H_

#include <oadrive_world/TrafficSign.h>
#include "ros_oadrive/TrafficSign.h"

namespace ros_oadrive {

class TrafficSignConverter {
 public:
  static TrafficSign toMessage(const oadrive::world::TrafficSign &traffic_sign);

  static oadrive::world::TrafficSign fromMessage(const TrafficSign &traffic_sign_msg);

  static oadrive::world::TrafficSignType getTypeFromId(int id);
};

}  // namespace ros_oadrive

#endif
