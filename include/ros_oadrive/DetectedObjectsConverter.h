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
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
 * \date    2017
 */
//----------------------------------------------------------------------

#ifndef ROS_OADRIVE_DETECTED_OBJECTS_CONVERTER_H_
#define ROS_OADRIVE_DETECTED_OBJECTS_CONVERTER_H_

#include "ros_oadrive/DetectedObjects.h"
#include "oadrive_obstacle/ObjectDetector.h"

namespace ros_oadrive {

class DetectedObjectsConverter {
 public:
  static DetectedObjects toMessage(const oadrive::obstacle::DetectedObjects &objectList);

  static oadrive::obstacle::DetectedObjects fromMessage(const DetectedObjects &DetectedObjects_msg);
};

}  // namespace ros_oadrive

#endif 
