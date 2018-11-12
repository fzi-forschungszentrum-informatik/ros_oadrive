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

#ifndef ROS_OADRIVE_MULTI_TRAJECTORY_CONVERTER_H_
#define ROS_OADRIVE_MULTI_TRAJECTORY_CONVERTER_H_

#include <oadrive_core/Trajectory2d.h>
#include <oadrive_world/MultiTrajectory.h>
#include "ros_oadrive/MultiTrajectory.h"

// TODO(kolja): Remove include of oadrive_core/Trajectory2d. Find out why CMake
// has no access to other header files.
//
// oadrive::core is missing when trying to look for Trajectory2d in the
// oadrive_worldMultiTrajectory.h file.

namespace ros_oadrive {

class MultiTrajectoryConverter {
 public:
  static MultiTrajectory toMessage(const oadrive::world::MultiTrajectory &traj);

  static oadrive::world::MultiTrajectory fromMessage(const MultiTrajectory &traj);
};

}  // namespace ros_oadrive

#endif /* ROS_OADRIVE_MULTI_TRAJECTORY_CONVERTER_H_ */
