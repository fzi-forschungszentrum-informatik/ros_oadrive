#include <gtest/gtest.h>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_world/MultiTrajectory.h>
#include "ros_oadrive/MultiTrajectory.h"
#include "ros_oadrive/MultiTrajectoryConverter.h"

TEST(MultiTrajectoryConverterTest, toMessage) {
  ros_oadrive::MultiTrajectory mt;
  oadrive::world::MultiTrajectory obj;
}

TEST(MultiTrajectoryConverterTest, fromMessage) {
  // TODO
}
