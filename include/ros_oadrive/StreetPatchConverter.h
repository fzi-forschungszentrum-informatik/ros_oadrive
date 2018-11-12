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
 * \author  Kolja Esders <esders@fzi.de>
 * \date    2017
 * 
 * \author  Fabian Dürr
 * \date    2017
 */
//----------------------------------------------------------------------

#ifndef ROS_OADRIVE_STREET_PATCH_CONVERTER_H_
#define ROS_OADRIVE_STREET_PATCH_CONVERTER_H_

#include <oadrive_lanedetection/RoadPatching/PatchHypothesis.h>
#include <oadrive_world/Patch.h>

#include "ros_oadrive/StreetPatch.h"

namespace ros_oadrive {

class StreetPatchConverter {
 public:
  static StreetPatch toMessage(const oadrive::lanedetection::PatchHypothesis &obj);

  static oadrive::world::PatchPtr fromMessage(const StreetPatch &msg);
};

}  // namespace ros_oadrive

#endif /* ROS_OADRIVE_STREET_PATCH_CONVERTER_H_ */
