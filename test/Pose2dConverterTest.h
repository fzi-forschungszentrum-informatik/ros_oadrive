#ifndef POSE_2D_CONVERTER_TEST_H_
#define POSE_2D_CONVERTER_TEST_H_

#include <gtest/gtest.h>
#include "geometry_msgs/Pose2D.h"
#include "oadrive_core/Pose.h"

namespace ros_oadrive {

class Pose2dConverterTest : public testing::Test {
 public:
  Pose2dConverterTest();

  static bool areEqual(const geometry_msgs::Pose2D &pose_msg, const oadrive::core::Pose2d &pose_obj);

 protected:
  geometry_msgs::Pose2D m_pose_msg;

  oadrive::core::Pose2d m_pose_obj;
};

}  // namespace ros_oadrive

#endif /* POSE_2D_CONVERTER_TEST_H_ */
