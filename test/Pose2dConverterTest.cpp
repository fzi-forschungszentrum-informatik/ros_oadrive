#include "Pose2dConverterTest.h"
#include <gtest/gtest.h>
#include "geometry_msgs/Pose2D.h"
#include "oadrive_core/Pose.h"
#include "ros_oadrive/Pose2dConverter.h"

namespace ros_oadrive {

Pose2dConverterTest::Pose2dConverterTest() {
  // Populate object and a message with identical data.
  double x = rand() % 10;
  double y = rand() % 10;
  double yaw = rand() % 10;
  m_pose_msg.x = x;
  m_pose_msg.y = y;
  m_pose_msg.theta = yaw;

  m_pose_obj.translation()[0] = x;
  m_pose_obj.translation()[1] = y;
  oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromOrientationRPY(
      m_pose_obj, yaw);
}

bool Pose2dConverterTest::areEqual(const geometry_msgs::Pose2D &pose_msg,
                                   const oadrive::core::Pose2d &pose_obj) {
  return pose_msg.x == pose_obj.translation()[0] &&
         pose_msg.y == pose_obj.translation()[1] &&
         pose_msg.theta ==
             oadrive::core::PoseTraits<oadrive::core::Pose2d>::yaw(pose_obj);
}

TEST_F(Pose2dConverterTest, toMessage) {
  const geometry_msgs::Pose2D conversion_result =
      Pose2dConverter::toMessage(m_pose_obj);
  EXPECT_TRUE(Pose2dConverterTest::areEqual(conversion_result, m_pose_obj));
}

TEST_F(Pose2dConverterTest, fromMessage) {
  const oadrive::core::Pose2d conversion_result =
      Pose2dConverter::fromMessage(m_pose_msg);
  EXPECT_TRUE(Pose2dConverterTest::areEqual(m_pose_msg, conversion_result));
}

}  // namespace ros_oadrive
