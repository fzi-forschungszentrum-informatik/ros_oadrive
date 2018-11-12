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
 * \date    2017-8-31
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Shuxiao Ding <ding@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------
#include "ros/ros.h"
#include "oadrive_util/Config.h"
#include "oadrive_obstacle/ObjectDetector.h"
#include "oadrive_obstacle/ObjectTracker.h"
#include "oadrive_util/CoordinateConverter.h"
#include "ros_oadrive/DetectedObjects.h"
#include "ros_oadrive/DetectedObjectsConverter.h"
#include "ros_oadrive/TrackedObjects.h"
#include "ros_oadrive/TrackedObjectsConverter.h"
#include "ros_oadrive/MarkerPosition.h"
#include "ros_oadrive/EmergencyBrake.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseArray.h"
#include "ros_oadrive/TrafficSign.h"
#include "ros_oadrive/TrafficSignConverter.h"
#include "ros_oadrive/Event.h"


#include "oadrive_obstacle/ProcessUS.h"

using namespace oadrive::obstacle;

/*!
   \brief ros node to communicate with the object tracker
   \brief listens to all object detections coming from YOLO, US, traffic sign detections and (not implemented yet) depth image detections
   \brief calls oadrive::obstacle::ObjectTracker to handel all detections
*/
class ObjectTrackerNode 
{
  public:
    ObjectTrackerNode(ros::NodeHandle node_handle, std::string configFile, std::string us_configPath)
      : mNode(node_handle)
      , mLoopRate(40)
      , mCoordinateConverter(configFile)
      , mObjectTracker(&mCoordinateConverter)
      , mCurrentDetectedObjects()
      , mCurrentCarPose()
      , mEmergencyBrake(false)
      , mProcessUS(us_configPath)
    {
      mDetectedObjects_sub = mNode.subscribe("/aadc/object_detection/detected_objects", 1, &ObjectTrackerNode::callBackDetectedObjects, this);
      mMarkerPosition_sub = mNode.subscribe("/aadc/marker_position", 1, &ObjectTrackerNode::callBackMarkerPosition, this);
      mEmergencyBrake_sub = mNode.subscribe("/aadc/statemachine/emergency_brake", 1, &ObjectTrackerNode::callBackEmergencyBrake, this);
      mTrafficSign_sub = mNode.subscribe("/aadc/objects/traffic_sign", 1, &ObjectTrackerNode::callBackTrafficSign, this);
      mEvent_sub = mNode.subscribe("/aadc/planning/event",1,&ObjectTrackerNode::callBackEvent, this);


      mTrackedObjects_pub = mNode.advertise<ros_oadrive::TrackedObjects>("/aadc/object_tracking/tracked_objects", 1);
      mUS_position_pub = mNode.advertise<geometry_msgs::PoseArray>("/aadc/object_detection/US_objects", 1);
    }
    ~ObjectTrackerNode()
    {

    }
    void run()
    {
      while (ros::ok())
      {
        ros::spinOnce();
        mTrackedObjects = mObjectTracker.getTrackedObjects(mCurrentCarPose);

        ros_oadrive::TrackedObjects trackedObjects_msg = ros_oadrive::TrackedObjectsConverter::toMessage(mTrackedObjects);
        mTrackedObjects_pub.publish(trackedObjects_msg);
        mLoopRate.sleep();
      }
    }

  private:    
    /*!
       \brief calls tracking of objects detected by objectDetector
       \param global pose of detected objects
    */
    void callBackDetectedObjects(const ros_oadrive::DetectedObjects detectedObjects_msg)
    {
      mCurrentDetectedObjects = ros_oadrive::DetectedObjectsConverter::fromMessage(detectedObjects_msg);

      mObjectTracker.trackObjects(mCurrentDetectedObjects);

    }
    /*!
       \brief save current position to transfrom coordinates, poses
       \param current car pose
    */
    void callBackMarkerPosition(const ros_oadrive::MarkerPosition pose_msg)
    {
      mCurrentCarPose.setX(pose_msg.pose.x);
      mCurrentCarPose.setY(pose_msg.pose.y);
      mCurrentCarPose.setYaw(pose_msg.pose.theta);
    }

    /*!
       \brief Preliminary US handling: Calculate pose of object that caused emergency brake and track it
       \param emergency event + US ids
    */
    void callBackEmergencyBrake(const ros_oadrive::EmergencyBrake emergencyBrake_msg)
    {
      mEmergencyBrake = emergencyBrake_msg.emergency_brake;
      if(mEmergencyBrake)
      {
        // calculate position of every US id that triggered EB and call ObjectTracker
        std::vector<ExtendedPose2d> US_poseList;
        for(unsigned int i = 0; i < emergencyBrake_msg.US_id.size(); i++)
        {
          ExtendedPose2d US_pose_ego = mProcessUS.transformToCar(emergencyBrake_msg.US_id[i], emergencyBrake_msg.US_distance[i]);
          ExtendedPose2d US_pose_odo = mCoordinateConverter.car2World(mCurrentCarPose, US_pose_ego);
          US_poseList.push_back(US_pose_odo);
        }
        if (US_poseList.size() > 0) {
          mObjectTracker.trackUS_obstacle(US_poseList);
        }
      }
    }
    
    /*!
       \brief Traffic signs are currently "tracked" using the objectTracker
       \param trafficSigns
    */
    void callBackTrafficSign(const ros_oadrive::TrafficSign trafficSign_msg)
    {
      oadrive::world::TrafficSign trafficSign = ros_oadrive::TrafficSignConverter::fromMessage(trafficSign_msg);
      ExtendedPose2d trafficSignPose_ego = trafficSign.getPose();
      //add camera offset to car origin (center of rear wheels) 	
      trafficSignPose_ego.setX(trafficSignPose_ego.getX());
      trafficSignPose_ego.setY(-trafficSignPose_ego.getY());
      ExtendedPose2d trafficSignPose_odo = mCoordinateConverter.car2World(mCurrentCarPose, trafficSignPose_ego);
      // std::cout << trafficSignPose_odo.getX() << " " << trafficSignPose_odo.getY() << " " << trafficSignPose_odo.getYaw() <<  std::endl;
      trafficSign.setPose(trafficSignPose_odo);
      mObjectTracker.trackTrafficSign(trafficSign);
    }
    /*!
       \brief reset function for object tracker
       \param event_msg which can trigger the reset command
    */
    void callBackEvent(const ros_oadrive::Event &event_msg)
    {
      if(event_msg.type == "RESET")
      {
        ROS_INFO("Reset ObjectTracker!");
        mObjectTracker.reset();
      }
    }
    //members
    ros::NodeHandle mNode;
    ros::Rate mLoopRate;
    DetectedObjects mCurrentDetectedObjects;
    TrackedObjects mTrackedObjects;
    oadrive::util::CoordinateConverter mCoordinateConverter;
    ObjectTracker mObjectTracker;
    ExtendedPose2d mCurrentCarPose;
    bool mEmergencyBrake;
    ProcessUS mProcessUS;

    
    //publishers
    ros::Publisher mTrackedObjects_pub;
    ros::Publisher mUS_position_pub;
  
    //subscribers
    ros::Subscriber mDetectedObjects_sub;
    ros::Subscriber mMarkerPosition_sub;
    ros::Subscriber mEmergencyBrake_sub;
    ros::Subscriber mUS_sub;
    ros::Subscriber mTrafficSign_sub;
    ros::Subscriber mEvent_sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ObjectTrackerNode");
  ros::NodeHandle nh("~");


  // processUS.loadCalPoints("/home/aadc2017/bagfiles/US.yml");


  std::string config_folder;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  std::cout << "Config folder: " << config_folder << std::endl;
  std::string mConfigPath(oadrive::util::Config::setConfigPath(config_folder, car_name));
  std::string birdViewCalFile((boost::filesystem::path(config_folder) / car_name / "BirdviewCal.yml").string());
  std::string US_configPath((boost::filesystem::path(config_folder) / "US_CalPoints.yml").string());
  std::cout << "Config file: " << birdViewCalFile << std::endl;

  ObjectTrackerNode rosNode(nh, birdViewCalFile, US_configPath);
  rosNode.run();
  return 0;
}

