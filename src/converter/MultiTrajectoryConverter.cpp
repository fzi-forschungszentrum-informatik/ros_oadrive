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

#include "ros_oadrive/MultiTrajectoryConverter.h"

#include <oadrive_world/MultiTrajectory.h>
#include "ros_oadrive/MultiTrajectory.h"

namespace ros_oadrive {

MultiTrajectory MultiTrajectoryConverter::toMessage(
    const oadrive::world::MultiTrajectory &obj) {
  std::vector<Trajectory2d> trajectories_msg;

  for (auto &traj : obj.trajectories) {
    Trajectory2d traj_msg;
    traj_msg.curvature_available = traj.curvatureAvailable();
    traj_msg.is_forward_trajectory = traj.isForwardTrajectory();
    traj_msg.velocity_available = traj.velocityAvailable();
    std::vector<ExtendedPose2d> trajectory_points_msg;

    for (size_t i = 0; i < traj.size(); i++) {
      ExtendedPose2d extended_pose_msg;
      extended_pose_msg.pose.x = traj[i].getX();
      extended_pose_msg.pose.y = traj[i].getY();
      extended_pose_msg.pose.theta = traj[i].getYaw();

      if (traj_msg.curvature_available)
        extended_pose_msg.curvature = traj[i].getCurvature();
      if (traj_msg.velocity_available)
        extended_pose_msg.velocity = traj[i].getVelocity();
      trajectory_points_msg.push_back(extended_pose_msg);
    }

    traj_msg.trajectory = trajectory_points_msg;
    trajectories_msg.push_back(traj_msg);
  }

  MultiTrajectory multitraj_msg;
  multitraj_msg.trajectories = trajectories_msg;

  return multitraj_msg;
}

oadrive::world::MultiTrajectory MultiTrajectoryConverter::fromMessage(
    const MultiTrajectory &msg) {
  oadrive::world::MultiTrajectory obj;

  for (auto &traj_msg : msg.trajectories) {
    oadrive::core::Trajectory2d traj;
    traj.curvatureAvailable() = traj_msg.curvature_available;
    traj.isForwardTrajectory() = traj_msg.is_forward_trajectory;
    traj.velocityAvailable() = traj_msg.velocity_available;

    for (size_t i = 0; i < traj_msg.trajectory.size(); i++) {
      oadrive::core::ExtendedPose2d extended_pose;

      extended_pose.setX(traj_msg.trajectory[i].pose.x);
      extended_pose.setY(traj_msg.trajectory[i].pose.y);
      extended_pose.setYaw(traj_msg.trajectory[i].pose.theta);

      if (traj_msg.curvature_available)
        extended_pose.setCurvature(traj_msg.trajectory[i].curvature);
      if (traj_msg.velocity_available)
        extended_pose.setVelocity(traj_msg.trajectory[i].velocity);
      traj.push_back(extended_pose);
    }
    obj.trajectories.push_back(traj);
  }

  return obj;
}

}  // namespace ros_oadrive
