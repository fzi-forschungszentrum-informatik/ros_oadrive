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
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 */
//----------------------------------------------------------------------

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <ros_oadrive/TrafficSign.h>
#include <ros_oadrive/DetectedObjects.h>
#include <ros_oadrive/TrackedObjectsConverter.h>
#include <ros_oadrive/EventRegion.h>


using namespace ros_oadrive;

class ObjectVisualizationNode {
protected:
    ros::Publisher mPubDetectedSign;
    ros::Publisher mPubSign;
    ros::Publisher mPubEventRegion;

public:
    int run() {
        // Init ros
        ros::NodeHandle nh("~");
        mPubDetectedSign = nh.advertise<visualization_msgs::Marker>("/aadc/visualization/detected_signs", 20);
        mPubSign = nh.advertise<visualization_msgs::Marker>("/aadc/visualization/tracked_objects", 20);
        mPubEventRegion = nh.advertise<visualization_msgs::Marker>("/aadc/visualization/event_regions", 20);
        ros::Subscriber subTrafficSign = nh.subscribe("/aadc/objects/traffic_sign", 5, &ObjectVisualizationNode::callbackTrafficSign, this);
        ros::Subscriber subTracker = nh.subscribe("/aadc/object_tracking/tracked_objects", 2, &ObjectVisualizationNode::callbackTracker, this);
        ros::Subscriber subDetector = nh.subscribe("/aadc/object_detection/detected_objects", 2, &ObjectVisualizationNode::callbackDetector, this);
        
        ros::Subscriber subEventRegion = nh.subscribe("/aadc/planning/event_region", 100, &ObjectVisualizationNode::eventRegionCallback, this);

        ros::Rate loop_rate(30);

        // Init controller
        while (ros::ok()) {            
            // Ros lifecycle
            ros::spinOnce();
            loop_rate.sleep();
        }

        return 0;
    }

    // Directly publishes traffic signs from the bridge
    void callbackTrafficSign(const ros_oadrive::TrafficSign trafficSign_msg)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "oriented_car";
        marker.header.stamp = ros::Time::now();
        marker.ns = "";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1);
        marker.pose.position.x = trafficSign_msg.object.pose.x;
        marker.pose.position.y = -trafficSign_msg.object.pose.y; // TODO: Remove this as soon as the bridge sends correct values
        marker.pose.position.z = 0.25;

        marker.color.a = 1.0;  // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.5;

        mPubDetectedSign.publish(marker);
    }

    void callbackDetector(const DetectedObjects::ConstPtr &detectedObjects) {
        int id = 100;
        for(auto object: detectedObjects->persons){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "local";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = id;
            marker.pose.position.x = object.object.pose.x;
            marker.pose.position.y = object.object.pose.y;
            marker.pose.position.z = 0.15;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.3;
            marker.lifetime = ros::Duration(1);

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            mPubSign.publish(marker);
            id++;
        }
    }

    // publishes traffic signs from the object tracker
    void callbackTracker(const TrackedObjects::ConstPtr &trackedObjects)
    {
        int id = 1;
        oadrive::obstacle::TrackedObjects objects = TrackedObjectsConverter::fromMessage(*trackedObjects);
 
        for (auto & sign : objects.trafficSigns)  {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "local";
            marker.header.stamp = ros::Time::now();
            marker.ns = "";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(1);
            marker.pose.position.x = sign.getX();
            marker.pose.position.y = sign.getY();
            marker.pose.position.z = 0.25;

            marker.color.a = 1.0;  // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.scale.x = 0.08;
            marker.scale.y = 0.08;
            marker.scale.z = 0.5;

            mPubSign.publish(marker);
        }

        for(auto object: trackedObjects->cars){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "local";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = id;
            marker.pose.position.x = object.object.pose.x;
            marker.pose.position.y = object.object.pose.y;
            marker.pose.position.z = 0.15;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.3;
            marker.lifetime = ros::Duration(1);

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            mPubSign.publish(marker);
            id++;
        }

        for(auto object: trackedObjects->persons){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "local";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = id;
            marker.pose.position.x = object.object.pose.x;
            marker.pose.position.y = object.object.pose.y;
            marker.pose.position.z = 0.15;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.3;
            marker.lifetime = ros::Duration(1);

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            mPubSign.publish(marker);
            id++;
        }
    }

    void eventRegionCallback(const ros_oadrive::EventRegion::ConstPtr &msg) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 100 + msg->uid;
        marker.lifetime = ros::Duration(10.0);
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = msg->pose.x;
        marker.pose.position.y = msg->pose.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.f;
        marker.pose.orientation.y = 0.f;
        marker.pose.orientation.z = sin(msg->pose.theta / 2.f);
        marker.pose.orientation.w = cos(msg->pose.theta / 2.f);

        marker.scale.x = msg->length;
        marker.scale.y = msg->width;
        marker.scale.z = 0.01;

        marker.color.a = 0.5;  // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        mPubEventRegion.publish(marker);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ObjectVisualizationNode");
    ObjectVisualizationNode rosNode;
    return rosNode.run();
}
