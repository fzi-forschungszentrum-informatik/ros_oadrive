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

#ifndef ROS_OADRIVE_TRACKED_OBJECTS_CONVERTER_H_
#define ROS_OADRIVE_TRACKED_OBJECTS_CONVERTER_H_

#include "ros_oadrive/TrackedObjects.h"
#include "oadrive_obstacle/ObjectTracker.h"

namespace ros_oadrive {

class TrackedObjectsConverter {
 public:
  static TrackedObjects toMessage(const oadrive::obstacle::TrackedObjects &objectList);

  static oadrive::obstacle::TrackedObjects fromMessage(const TrackedObjects &TrackedObjects_msg);
};

}  // namespace ros_oadrive

#endif 
