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
 *
 * \author  Kolja Esders <esders@fzi.de>
 * \date    2017
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "ros_oadrive/TrackedObjectConverter.h"
#include "ros/ros.h"

namespace ros_oadrive {

ros_oadrive::TrackedObject TrackedObjectConverter::toMessage(const oadrive::obstacle::TrackedObject &object) {
  ros_oadrive::TrackedObject object_msg;

  std::vector<float> vec(object.poseErrorCovariance.data(),
                         object.poseErrorCovariance.data() + object.poseErrorCovariance.size());

  object_msg.pose.x = object.pose.getX();
  object_msg.pose.y = object.pose.getY();
  object_msg.poseErrorCovariance = vec;
  object_msg.width = object.width;
  object_msg.widthErrorCovariance = object.widthErrorCovariance;
  object_msg.filterID = object.filterID;
  object_msg.numberOfMeasurements = object.numberOfMeasurements;

  return object_msg;
}

oadrive::obstacle::TrackedObject TrackedObjectConverter::fromMessage(
    const ros_oadrive::TrackedObject &object_msg) {
  oadrive::obstacle::TrackedObject object;

  std::vector<float> vec = object_msg.poseErrorCovariance;
  if(vec.size() == 16)
  {
    int n = 4;
    float *v = &vec[0];
    Eigen::Map<Eigen::MatrixXf> matrix(v, n, n);
    object.poseErrorCovariance = matrix;
  }
  else if(vec.size() != 4){
    ROS_WARN("poseErrorCovariance element count != 16");
  }
  object.pose.setX(object_msg.pose.x);
  object.pose.setY(object_msg.pose.y);

  object.width = object_msg.width;
  object.widthErrorCovariance = object_msg.widthErrorCovariance;
  object.filterID = object_msg.filterID;
  object.numberOfMeasurements = object_msg.numberOfMeasurements;

  return object;
}

}  // namespace ros_oadrive
