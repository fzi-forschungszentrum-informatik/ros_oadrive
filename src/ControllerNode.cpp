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
#include <oadrive_util/Config.h>
#include <oadrive_world/TrajectoryDatabase.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <oadrive_control/DriverModule.h>
#include <oadrive_control/RampUSController.h>
#include "oadrive_obstacle/ObjectTracker.h"

#include "ros_oadrive/Event.h"
#include "ros_oadrive/ControllerCommand.h"
#include "ros_oadrive/MarkerPosition.h"
#include "ros_oadrive/MultiTrajectory.h"
#include <ackermann_msgs/AckermannDrive.h>
#include "ros_oadrive/TrackedObjects.h"
#include "ros_oadrive/TrackedObjectsConverter.h"
#include <ros_oadrive/Ultrasonic.h>
#include <ros_oadrive/StateChange.h>
#include <oadrive_world/StateMachine.h>


// if set, car positions are transformed to local coordinates
#define LOCAL_TRAJECTORIES true

using namespace oadrive::core;
using namespace oadrive::world;
using namespace oadrive::control;
using namespace oadrive::obstacle;

/*!
   * \brief callBackControllerCommand: node to communicate with the lateral and speed controller!
   */

class ControllerNode
{
public:
  ControllerNode(ros::NodeHandle node_handle)
      : mNode(node_handle)
      , mLoopRate(80)
      , mDriver()
      , mCurrentCarPoseGlobal()
      , mCurrentCarPoseLocal()
      , mControl_msg()
      , mCarPoseOrigin()
  {
    // contains speed (m/s) and steering (radian) output of the controller
    mControl_pub = mNode.advertise<ackermann_msgs::AckermannDrive>("/aadc/control/car", 1);
    // only for visualization
    mLocalCarPose_pub = mNode.advertise<ros_oadrive::ExtendedPose2d>("/DriverModule/LocalCarPose", 1);
    // returns controller status to the statemachine e.g. "trajectory reached = true"
    mEvent_pub = mNode.advertise<ros_oadrive::Event>("/aadc/planning/event", 1);

    // command from the statemachine e.g. drive, halt, follow car etc.
    mStatus_sub = mNode.subscribe("/aadc/driver_module/command", 1, &ControllerNode::callBackControllerCommand, this);
    // position, orientation and speed from ADTF
    mMarkerPosition_sub = mNode.subscribe("/aadc/marker_position", 1, &ControllerNode::callBackMarkerPosition, this);
    // trajectory from the planning node
    mMultiTrajectory_sub = mNode.subscribe("/aadc/planning/trajectory",1,&ControllerNode::callBackMultiTrajectory, this);
    // Currently tracked objects. Message contains velocity and pose from tracked cars so we could follow them i.e. controll the speed
    mTrackedObjects_sub = mNode.subscribe("/aadc/object_tracking/tracked_objects", 1, &ControllerNode::callBackTrackedObjects, this);
    
    mState_sub = mNode.subscribe("/aadc/statemachine/state", 10, &ControllerNode::stateCallback, this);
    mUS_sub = mNode.subscribe("/aadc/ultrasonic", 10, &ControllerNode::usCallback, this);
  }

  ~ControllerNode()
  {

  }
  /*!
     * \brief run: controller loop. Writes and publishes ros messages
     */
  void run()
  {
    while (ros::ok())
    {
      if (mCurrentState != StateMachine::State::DRIVING_RAMP) {
        mControl_msg.steering_angle = mDriver.getSteeringAngle()*M_PI/180.0; 
      } else {
        mControl_msg.steering_angle = mUSLateralController.update();
      }
      mControl_msg.speed = mDriver.getSpeed();
      mControl_pub.publish(mControl_msg);

      if ((mDriver.getNumberOfRemainingTrajectories() <= 1 && mDriver.getNumberOfRemainingTrajectoryPoints() < 3) || mDriver.checkEndOfTrajectoryFlag()) {
        ros_oadrive::Event event;
        event.type = "TRAJECTORY_FINISHED";
        mEvent_pub.publish(event);
      }

#if LOCAL_TRAJECTORIES
      mCurrentCarPoseLocal_msg.pose.x = mCurrentCarPoseLocal.getX();
      mCurrentCarPoseLocal_msg.pose.y = mCurrentCarPoseLocal.getY();
      mCurrentCarPoseLocal_msg.pose.theta = mCurrentCarPoseLocal.getYaw();
      mCurrentCarPoseLocal_msg.velocity = mCurrentCarPoseLocal.getVelocity();
#else
      mCurrentCarPoseLocal_msg.pose.x = mCurrentCarPoseGlobal.getX();
      mCurrentCarPoseLocal_msg.pose.y = mCurrentCarPoseGlobal.getY();
      mCurrentCarPoseLocal_msg.pose.theta = mCurrentCarPoseGlobal.getYaw();
      mCurrentCarPoseLocal_msg.velocity = mCurrentCarPoseGlobal.getVelocity();
#endif
      mLocalCarPose_pub.publish(mCurrentCarPoseLocal_msg);
      ros::spinOnce();
      mLoopRate.sleep();
      }
  }

private:
  /*!
     * \brief callBackControllerCommand: listens to the statemachine, and sets the status for the controller e.g. drive or halt
     * \param command_msg statemachine command for the controller
     */
  void callBackControllerCommand(const ros_oadrive::ControllerCommand::ConstPtr& command_msg){
    // call speed but limit speed to 4.0m/s
    mDriver.setMaxSpeed(command_msg->target_speed);

    // check driver command
    if(!command_msg->drive)
    {
      mDriver.halt(command_msg->active_brake);
    }
    else
    {
      mDriver.drive();
    }
  }

  /*!
     * \brief callBackMarkerPosition: every time our position is updated, we update the controller.
     * \param  markerPosition: Given by ADTF: x,y, speed, orientation, some kind of uncertainty radius (not used)
     */
  void callBackMarkerPosition(const ros_oadrive::MarkerPosition::ConstPtr& markerPosition_msg){
    mCurrentCarPoseGlobal.setX(markerPosition_msg->pose.x);
    mCurrentCarPoseGlobal.setY(markerPosition_msg->pose.y);
    mCurrentCarPoseGlobal.setYaw(markerPosition_msg->pose.theta);
    mCurrentCarPoseGlobal.setVelocity(markerPosition_msg->speed);

#if LOCAL_TRAJECTORIES
    //if we get local trajs i.e. if the points are given relative to the car origin then: transform car position to local coordinates as well
    float relativeX = mCurrentCarPoseGlobal.getX()-mCarPoseOrigin.getX();
    float relativeY = mCurrentCarPoseGlobal.getY()-mCarPoseOrigin.getY();
    // float localX = relativeX*cos(mCarPoseOrigin.getYaw()) - relativeY*sin(mCarPoseOrigin.getYaw());
    // float localY = relativeY*cos(mCarPoseOrigin.getYaw()) + relativeX*sin(mCarPoseOrigin.getYaw());
    float localX = relativeX*cos(mCarPoseOrigin.getYaw()) + relativeY*sin(mCarPoseOrigin.getYaw());
    float localY = relativeY*cos(mCarPoseOrigin.getYaw()) - relativeX*sin(mCarPoseOrigin.getYaw());
    float localYaw = mCurrentCarPoseGlobal.getYaw()-mCarPoseOrigin.getYaw();
    mCurrentCarPoseLocal.setYaw(localYaw);
    mCurrentCarPoseLocal.setX(localX);
    mCurrentCarPoseLocal.setY(localY);

    mCurrentCarPoseLocal.setVelocity(markerPosition_msg->speed);
    mDriver.update(mCurrentCarPoseLocal);
#else
    // has never been tested
    ROS_INFO("Global Position: x: %f y: %f yaw: %f Local Position: x: %f y: %f yaw: %f",mCurrentCarPoseGlobal.getX(),mCurrentCarPoseGlobal.getY(),mCurrentCarPoseGlobal.getYaw());
    mDriver.update(mCurrentCarPoseGlobal);
#endif
  }

  /*!
     * \brief callBackMultiTrajectory: Every time we get a new trajectory, the current car pose is set as the origin and the controller receives the new traj
     * \param command_msg statemachine command for the controller
     */
  void callBackMultiTrajectory(const ros_oadrive::MultiTrajectory &multiTraj_msg)
  {
    ROS_INFO("Received new multiTrajectory!");
    mCarPoseOrigin = mCurrentCarPoseGlobal;
    mCurrentMultiTrajectory = ROS_MSG2MultiTrajectory(multiTraj_msg);
    mDriver.setMultiTrajectory(mCurrentMultiTrajectory);
  }

  /*!
     * \brief callBackTrackedObjects: Used for car following but never tested!!
     * \param trackedObjects Struct containing pose + speed of tracked cars
     */
  void callBackTrackedObjects(const ros_oadrive::TrackedObjects &trackedObjects_msg)
  {
    TrackedObjects trackedObjects = ros_oadrive::TrackedObjectsConverter::fromMessage(trackedObjects_msg);
    mTrackedCars = trackedObjects.cars;

    //associate tracked car id for car following
    if(trackedObjects.follow_car_id != -1)
    {
      bool foundMatchingId = false;
      for(TrackedCar & car : mTrackedCars)
      {
        if(car.object.filterID == trackedObjects.follow_car_id)
        {
          foundMatchingId = true;
          
          ROS_WARN("Following %i!", trackedObjects.follow_car_id);
          mDriver.enableCarFollowing(car);
        }
      }
      if(!foundMatchingId)
      {
        ROS_WARN("No filter was found for car following and given id: %i!", trackedObjects.follow_car_id);
        mDriver.disableCarFollowing();
      }
    }
    else
    {
      mDriver.disableCarFollowing();
    }
  }

  void stateCallback(const ros_oadrive::StateChange::ConstPtr &msg)
  {
    auto prevState = static_cast<StateMachine::State>(msg->prevState);
    auto newState = static_cast<StateMachine::State>(msg->newState);

    mCurrentState = newState;
  }

  void usCallback(const ros_oadrive::Ultrasonic::ConstPtr& msg)
  {
    mUSLateralController.addUSSample(msg->sideLeft, msg->sideRight);
  }

  // old code. Converter classes should be used here...
  MultiTrajectory ROS_MSG2MultiTrajectory(const ros_oadrive::MultiTrajectory &multiTraj_msg)
  {
    MultiTrajectory multiTraj;
    for(size_t j = 0; j < multiTraj_msg.trajectories.size(); j++ )
    {
      ros_oadrive::Trajectory2d traj_msg = multiTraj_msg.trajectories.at(j);
      Trajectory2d traj;
      traj.curvatureAvailable() = traj_msg.curvature_available;
      traj.isForwardTrajectory() = traj_msg.is_forward_trajectory;
      traj.velocityAvailable() = traj_msg.velocity_available;
      for(size_t i = 0; i < traj_msg.trajectory.size(); i++ )
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

  //members
  ros::NodeHandle mNode;
  ros::Rate mLoopRate;
  DriverModule mDriver;
  RampUSController mUSLateralController;
  ExtendedPose2d mCurrentCarPoseLocal;
  ExtendedPose2d mCurrentCarPoseGlobal;
  ExtendedPose2d mCarPoseOrigin;
  MultiTrajectory mCurrentMultiTrajectory;
  std::vector<TrackedCar>mTrackedCars;

  StateMachine::State mCurrentState;

  //publishers
  ros::Publisher mControl_pub;
  ros::Publisher mLocalCarPose_pub;
  ros::Publisher mEvent_pub;

  //subscribers
  ros::Subscriber mStatus_sub;
  ros::Subscriber mMarkerPosition_sub;
  ros::Subscriber mMultiTrajectory_sub;
  ros::Subscriber mTrackedObjects_sub;
  ros::Subscriber mState_sub;
  ros::Subscriber mUS_sub;

  //msgs
  ackermann_msgs::AckermannDrive mControl_msg;
  ros_oadrive::ExtendedPose2d mCurrentCarPoseLocal_msg;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ControllerNode");
  ros::NodeHandle nh("~");

  std::string config_folder;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  std::cout << "Config folder: " << config_folder << std::endl;
  // Set config path for controller parameters
  std::string mConfigPath(oadrive::util::Config::setConfigPath(config_folder, car_name));

  ControllerNode rosNode(nh);
  rosNode.run();
  return 0;
}
