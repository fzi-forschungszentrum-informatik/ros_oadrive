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
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "oadrive_util/BirdViewConverter.h"
#include "oadrive_util/Config.h"
#include "ros/ros.h"
#include <ros_oadrive/ImagePosition.h>

/*!
   \brief ros node to convert basler image to birdview
*/
class BirdviewConverterNode {
 public:
  BirdviewConverterNode(ros::NodeHandle node_handle, std::string birdview_config_path)
      : m_node(node_handle), m_image_transport(node_handle) {
    m_birdview_converter.loadConfig(birdview_config_path);
    m_basler_image_sub = m_node.subscribe(
        "/aadc/imagepos/image", 1, &BirdviewConverterNode::callBackBaslerImage, this);
    m_birdview_pub = m_node.advertise<ros_oadrive::ImagePosition>("/aadc/front/birdview", 1);
    m_birdview_net_publisher = m_image_transport.advertise("/aadc/birdview", 1);
  }

  ~BirdviewConverterNode() {}

  void run() {
    ros::spin();
  }

  void callBackBaslerImage(const ros_oadrive::ImagePosition::ConstPtr& basler_image_msg) {
    cv::Mat basler_image = cv_bridge::toCvCopy(basler_image_msg->image, "mono8")->image;
    cv::Mat birdview = m_birdview_converter.transform(basler_image);

    std_msgs::Header header;
    header.stamp = basler_image_msg->image.header.stamp;

    sensor_msgs::ImageConstPtr birdview_image_msg =
      cv_bridge::CvImage(header, "mono8", birdview).toImageMsg();
    ros_oadrive::ImagePosition newImagePosition;
    newImagePosition.image = *birdview_image_msg;
    newImagePosition.pose = basler_image_msg->pose;

    m_birdview_net_publisher.publish(birdview_image_msg);
    m_birdview_pub.publish(newImagePosition);
  }

 private:
  // members
  ros::NodeHandle m_node;
  oadrive::util::BirdViewConverter m_birdview_converter;

  image_transport::ImageTransport m_image_transport;

  // publishers
  ros::Publisher m_birdview_pub;
 image_transport::Publisher m_birdview_net_publisher;

  // subscribers
  ros::Subscriber m_basler_image_sub;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "BirdviewConverterNode");
  ros::NodeHandle nh("~");

  std::string config_folder;
  std::string car_name;
  nh.param<std::string>("config_folder", config_folder, "");
  nh.param<std::string>("car_name", car_name, "");

  std::string config_path(oadrive::util::Config::setConfigPath(config_folder, car_name));
  std::string birdview_config_path(
      (boost::filesystem::path(config_folder) / car_name / "BirdviewCal.yml").string());
  BirdviewConverterNode ros_node(nh, birdview_config_path);
  ros_node.run();
  return 0;
}
