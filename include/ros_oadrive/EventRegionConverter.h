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

#ifndef ROS_OADRIVE_EVENT_REGION_CONVERTER_H_
#define ROS_OADRIVE_EVENT_REGION_CONVERTER_H_

#include <oadrive_world/EventRegion.h>
#include "ros_oadrive/EventRegion.h"

namespace ros_oadrive {

class EventRegionConverter {
 public:
  static EventRegion toMessage(const oadrive::world::EventRegion &obj);

  static oadrive::world::EventRegion fromMessage(const EventRegion &msg);
};

}  // namespace ros_oadrive

#endif /* ROS_OADRIVE_EVENT_REGION_CONVERTER_H_ */
