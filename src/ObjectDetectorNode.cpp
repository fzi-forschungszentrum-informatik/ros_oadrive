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
 * \date    2017-8-25
 * 
 * \author  Yimeng Zhu <yzhu@fzi.de>
 * \date    2018
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "ros/ros.h"
#include "oadrive_obstacle/ObjectDetector.h"
#include "oadrive_util/CoordinateConverter.h"
#include "oadrive_obstacle/ProcessDepth.h"
#include "oadrive_core/ExtendedPose2d.h"
#include <oadrive_util/Config.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "boost/filesystem.hpp"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include "ros_oadrive/MarkerPosition.h"
#include "ros_oadrive/DetectedObjects.h"
#include "ros_oadrive/DetectedObjectsConverter.h"

#include <visualization_msgs/Marker.h>




using namespace oadrive::obstacle;
using namespace oadrive::core;
using namespace oadrive::util;

/*!
   \brief ros node to communicate with the object detector. Processes bounding boxes found by YOLO.
   \brief ToDo: Find blobs in depth image. See old code in oadrive::obstacle::ProcessDepth
*/
class ObjectDetectorNode
{
  public:
    ObjectDetectorNode(ros::NodeHandle node_handle, std::string birdViewCalFile)
        : mNode(node_handle)
        , mImageTransport(mNode)
        , mLoopRate(15)
        , mObjectDetector(birdViewCalFile)
//        , mProcessDepth("", &mCoordinateConverterDepth)
        , mCurrentBoundingBoxList()
    {
      mObjectBoundingBoxes_sub = mNode.subscribe("/darknet_ros/bounding_boxes", 1, &ObjectDetectorNode ::callBackBoundingBoxes, this);
      mCarPose_sub = mNode.subscribe("/aadc/marker_position", 1, &ObjectDetectorNode::callBackCarPose, this);
//      mDepthImage_sub = mImageTransport.subscribe("/aadc/realsense/depth_raw", 1, &ObjectDetectorNode::callBackDepthImage, this);

      m_rivz_detected_object_pub = mNode.advertise<visualization_msgs::Marker>("/aadc/visualization/detected_objects", 1);
      mObjectList_pub = mNode.advertise<ros_oadrive::DetectedObjects>("/aadc/object_detection/detected_objects", 1);
    }
    ~ObjectDetectorNode()
    {

    }
    void run()
    {
      while (ros::ok())
      {
        ros::spinOnce();
        mLoopRate.sleep();
      }
    }
  private:
    /*!
       \brief read bounding boxes of cars or people and call findObjects
       \param boundingBoxes from YOLO
    */
    void callBackBoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boundingBoxes_msg)
    {
      std::vector<BoundingBox> boxList;
      for(auto & box_msg : boundingBoxes_msg->bounding_boxes)
      {
        BoundingBox box;
        box.Class = box_msg.Class;
        box.probability = box_msg.probability;
        box.xmin = box_msg.xmin;
        box.xmax = box_msg.xmax;
        box.ymin = box_msg.ymin;
        box.ymax = box_msg.ymax;
        boxList.push_back(box);
      }

      ros_oadrive::DetectedObjects objectList_msg = ros_oadrive::DetectedObjectsConverter::toMessage(mObjectDetector.findObjects(boxList, mCurrentCarPose));
      mObjectList_pub.publish(objectList_msg);
      publishDetectedObjectVisualization(mObjectDetector.findObjects(boxList, mCurrentCarPose));
    }
    /*!
       \brief save current pose to calculate object positions from images
       \param Car pose
    */
    void callBackCarPose(const ros_oadrive::MarkerPosition::ConstPtr& carPose_msg)
    {
      mCurrentCarPose.setX(carPose_msg->pose.x);
      mCurrentCarPose.setY(carPose_msg->pose.y);
      mCurrentCarPose.setYaw(carPose_msg->pose.theta);
    }

//    void callBackDepthImage(const sensor_msgs::ImageConstPtr& depthImage_msg)
//    {
//      cv::Mat depthImage = cv_bridge::toCvCopy(depthImage_msg, "bgr8")->image;
//      mProcessDepth.processDepthImageDebug(depthImage);
//      //cv::imshow("DepthImage",depthImage);
//      //cv::waitKey(1);
//    }

     void publishDetectedObjectVisualization(const oadrive::obstacle::DetectedObjects &objectList) 
    {
      int id = 0;
      for (auto detectedObj : objectList.persons)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.lifetime = ros::Duration(1.0);
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = detectedObj.object.pose.getX();
        marker.pose.position.y = detectedObj.object.pose.getY();
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.f;
        marker.pose.orientation.y = 0.f;


        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1.0;

        marker.color.a = 0.5;  // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    
        m_rivz_detected_object_pub.publish(marker);
        id++;
      }
      for (auto detectedObj : objectList.cars)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.lifetime = ros::Duration(1.0);
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = detectedObj.object.pose.getX();
        marker.pose.position.y = detectedObj.object.pose.getY();
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.f;
        marker.pose.orientation.y = 0.f;


        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 0.5;

        marker.color.a = 0.5;  // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    
        m_rivz_detected_object_pub.publish(marker);
        id++;
      }
    } 

    //members
    ros::NodeHandle mNode;
    image_transport::ImageTransport mImageTransport;
    ros::Rate mLoopRate;
    ObjectDetector mObjectDetector;
//    CoordinateConverter mCoordinateConverterDepth;
//    ProcessDepth mProcessDepth;

    std::vector<BoundingBox> mCurrentBoundingBoxList;
    ExtendedPose2d mCurrentCarPose;
    
    //publishers
    ros::Publisher mObjectList_pub;
    ros::Publisher m_rivz_detected_object_pub;
  
    //subscribers
    ros::Subscriber mCarPose_sub;
    ros::Subscriber mObjectBoundingBoxes_sub;
    image_transport::Subscriber mDepthImage_sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ObjectDetectorNode");
  ros::NodeHandle nh("~");

  std::string config_folder;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  std::cout << "Config folder: " << config_folder << std::endl;
  std::string mConfigPath(oadrive::util::Config::setConfigPath(config_folder, car_name));
  std::string birdViewCalFile((boost::filesystem::path(config_folder) / car_name / "BirdviewCal.yml").string());
 // std::string birdViewCalFileDepth((boost::filesystem::path(config_folder) / car_name / "BirdviewCal_realsense.yml").string());

//  std::string config_folder("/home/aadc2017/robot_folders/checkout/oadrive/ic_workspace/packages/oadrive/config");
//  std::string car_name("Abra");

//  std::string mConfigPath(oadrive::util::Config::setConfigPath(config_folder, car_name));
//  std::cout << "Config file: " << birdViewCalFile << std::endl;

  ObjectDetectorNode rosNode(nh,birdViewCalFile);
  rosNode.run();
  return 0;
}
