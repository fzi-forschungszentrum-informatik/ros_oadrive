/*! Ros dummy node to publish a trajectory. Just type the correspondig letter in the console to publish a trajectory.
 *  \brrief This can be very useful to test the controller to drive "manually"
    created by Robin Andlauer on June 2017
*/

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
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
 * \date    2017
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------
#include "ros/ros.h"
#include <oadrive_world/TrajectoryFactory.h>
#include "ros_oadrive/MultiTrajectory.h"
#include "ros_oadrive/MarkerPosition.h"

using namespace oadrive::core;
using namespace oadrive::world;


/*!
   \brief create custom trajectory. Simply create a vector of ExtendePose2d
*/
Trajectory2d createCustomTraj()
{
  Trajectory2d traj;
  // 7m forward
  float distance = 7.0f;
  float numberOfPoints = 5.0f;
  for ( float x = 0; x <= distance; x += distance/numberOfPoints )
  {
    ExtendedPose2d pose( x, 0, 0 );
    traj.push_back( pose );
  }
  return traj;
}

/*!
   \brief converts a MultiTrajectory into a ros_oadrive::MultiTrajectory msg format
   \brief outdated!!! Use converter class instead
*/
ros_oadrive::MultiTrajectory multiTrajectory2msg(MultiTrajectory multiTraj)
{
  std::vector<ros_oadrive::Trajectory2d> trajectories_msg;
  for(size_t j = 0; j < multiTraj.trajectories.size(); j++ ) // for each trajectory
  {
    Trajectory2d traj = multiTraj.trajectories.at(j);
    ros_oadrive::Trajectory2d traj_msg;
    traj_msg.curvature_available = traj.curvatureAvailable();
    traj_msg.is_forward_trajectory = traj.isForwardTrajectory();
    traj_msg.velocity_available = traj.velocityAvailable();
    std::vector<ros_oadrive::ExtendedPose2d> trajectoryPoints_msg;
    for(size_t i = 0; i < traj.size(); i++ ) // for each extendedPoint2d
    {
      ros_oadrive::ExtendedPose2d extendedPose_msg;
      extendedPose_msg.pose.x = traj[i].getX();
      extendedPose_msg.pose.y = traj[i].getY();
      extendedPose_msg.pose.theta = traj[i].getYaw();
      if(traj_msg.curvature_available)
        extendedPose_msg.curvature = traj[i].getCurvature();
      if(traj_msg.velocity_available)
        extendedPose_msg.velocity = traj[i].getVelocity();
      trajectoryPoints_msg.push_back(extendedPose_msg);
    }
    traj_msg.trajectory = trajectoryPoints_msg;
    trajectories_msg.push_back(traj_msg);
  }

  ros_oadrive::MultiTrajectory multiTraj_msg;
  multiTraj_msg.trajectories = trajectories_msg;
  return multiTraj_msg;
}
/*!
   \brief outdated!!! Use converter class instead
*/
MultiTrajectory msg2MultiTrajectory(ros_oadrive::MultiTrajectory multiTraj_msg)
{
  MultiTrajectory multiTraj;
  for(size_t j = 0; j < multiTraj_msg.trajectories.size(); j++ ) // for each trajectory
  {
    ros_oadrive::Trajectory2d traj_msg = multiTraj_msg.trajectories.at(j);
    Trajectory2d traj;
    traj.curvatureAvailable() = traj_msg.curvature_available;
    traj.isForwardTrajectory() = traj_msg.is_forward_trajectory;
    traj.velocityAvailable() = traj_msg.velocity_available;
    for(size_t i = 0; i < traj_msg.trajectory.size(); i++ ) // for each extendedPoint2d
    {
      ExtendedPose2d extendedPose;
      extendedPose.setX(traj_msg.trajectory[i].pose.x);
      extendedPose.setY(traj_msg.trajectory[i].pose.y);
      extendedPose.setYaw(traj_msg.trajectory[i].pose.theta);
      if(traj_msg.curvature_available)
        extendedPose.setCurvature(traj_msg.trajectory[i].curvature);
      if(traj_msg.velocity_available)
        extendedPose.setVelocity(traj_msg.trajectory[i].velocity);
      traj.push_back(extendedPose);
    }
    multiTraj.trajectories.push_back(traj);
  }
  return multiTraj;
}

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

Trajectory2d convertToBackwardsTraj(Trajectory2d traj)
{
  Trajectory2d backwardsTraj;
  backwardsTraj.isForwardTrajectory() = false;
  for(size_t i = 0; i < traj.size();i++)
  {
    ExtendedPose2d trajPoint;
    trajPoint.setX(-traj[i].getX());
    trajPoint.setY(traj[i].getY());
    backwardsTraj.push_back(trajPoint);
  }
  return backwardsTraj;
}

float m_yaw = 0;
void carPoseCallback(const ros_oadrive::MarkerPosition::ConstPtr &pose_msg) {
  m_yaw = pose_msg->pose.theta;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosDummyTrajectory");
  ros::NodeHandle n;
  ros::Publisher pub_trajectory = n.advertise<ros_oadrive::MultiTrajectory>("/aadc/planning/trajectory", 1);
  ros::Subscriber sub = n.subscribe("/aadc/marker_position", 1, &carPoseCallback);
  ros::Rate loop_rate(10);
  TrajectoryDatabase::load("/home/aadc/robot_folders/checkout/aadc18/ic_workspace/packages/oadrive/config/trajectories");
  while (ros::ok())
  {
    std::string inputTrajSelection;
    std::cout << "Publish new trajectory:" << std::endl;
    std::cout << "1: rectangle" << std::endl;
    std::cout << "2: halfCircle" << std::endl;
    std::cout << "3: uTurn" << std::endl;
    std::cout << "r3: uTurnLeft" << std::endl;
    std::cout << "4: oval" << std::endl;
    std::cout << "5: circle" << std::endl;
    std::cout << "6: 2x2 curve" << std::endl;
    std::cout << "w: forwardSmall" << std::endl;
    std::cout << "s: backwardSmall" << std::endl;
    std::cout << "d or dd: Right" << std::endl;
    std::cout << "a or aa: left" << std::endl;
    std::cout << "c: custom" << std::endl;
    std::cout << "Write '-' to drive backwards e.g. '-c'" << std::endl;
    std::cout << "Anything else: QUIT" << std::endl;

    std::cin >> inputTrajSelection;
    std::string trajSelection = inputTrajSelection;
    bool driveBackwards = false;
    if (trajSelection[0] == '-')
    {
      driveBackwards = true;
      trajSelection.erase(0,1);
    }

    ros::spinOnce();
    // Trajectory2d traj;
    // switch(str2int(trajSelection.c_str()))
    // {
    //   case str2int("1"):
    //     traj = TrajectoryFactory::generateTestTrajectory("rectangle");break;
    //   case str2int("2"):
    //     traj = TrajectoryFactory::generateTestTrajectory("halfCircle");break;
    //   case str2int("3"):
    //     traj = TrajectoryFactory::generateTestTrajectory("uTurn");break;
    //   case str2int("r3"):
    //     traj = TrajectoryFactory::generateTestTrajectory("uTurnLeft");break;
    //   case str2int("4"):
    //     traj = TrajectoryFactory::generateTestTrajectory("oval");break;
    //   case str2int("5"):
    //     traj = TrajectoryFactory::generateTestTrajectory("circle");break;
    //   case str2int("6"):
    //     traj = TrajectoryFactory::generateTestTrajectory("2x2curve");break;
    //   case str2int("dd"):
    //     traj = TrajectoryFactory::generateTestTrajectory("right");break;
    //   case str2int("aa"):
    //     traj = TrajectoryFactory::generateTestTrajectory("left");break;
    //   case str2int("d"):
    //     traj = TrajectoryFactory::generateTestTrajectory("sharpRight");break;
    //   case str2int("a"):
    //     traj = TrajectoryFactory::generateTestTrajectory("sharpLeft");break;
    //   case str2int("w"):
    //     traj = TrajectoryFactory::generateTestTrajectory("forwardSmall");break;
    //   case str2int("s"):
    //     traj = TrajectoryFactory::generateTestTrajectory("backwardsSmall");break;
    //   case str2int("c"):
    //     traj = createCustomTraj();break;
    //   default:
    //     return 0;
    // }
    // if(driveBackwards)
    //   traj = convertToBackwardsTraj(traj);
    // MultiTrajectory multiTraj = TrajectoryDatabase::getTrajectory("5pointturn");
    MultiTrajectory multiTraj = TrajectoryDatabase::getTrajectory("turn_slow");

    // multiTraj = TrajectoryFactory::rotateAndMoveTrajectory(multiTraj, ExtendedPose2d(0,0, M_PI));
    // multiTraj.trajectories.push_back(traj);

    // const float yaw = m_yaw;
    const float yaw = m_yaw - M_PI / 4;
    for (auto &traj : multiTraj.trajectories) {
      for (size_t i = 0; i < traj.size(); i++)
      {
        double x = traj.at(i).getX();
        double y = traj.at(i).getY();

        // Rotate point by angle of car pose:
        ExtendedPose2d rotated(x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw), 0);

        // Add car position:
        traj.at(i).setX(rotated.getX());
        traj.at(i).setY(rotated.getY());
        traj.at(i).setYaw(fmod(traj.at(i).getYaw() + yaw, 2 * M_PI));
      }
    }

    ros_oadrive::MultiTrajectory multiTraj_msg = multiTrajectory2msg(multiTraj);

    pub_trajectory.publish(multiTraj_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

