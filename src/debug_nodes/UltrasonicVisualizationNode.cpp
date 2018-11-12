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
#include <sensor_msgs/Range.h>
#include <ros_oadrive/Ultrasonic.h>


class UltrasonicVisualizationNode {
protected:
    ros::Publisher mPubUSSideLeft;
    ros::Publisher mPubUSSideRight;
    ros::Publisher mPubUSRearLeft;
    ros::Publisher mPubUSRearCenter;
    ros::Publisher mPubUSRearRight;

    tf::TransformBroadcaster* m_broadcaster;

    sensor_msgs::Range mMsg;

public:
    int run() {
        // Init ros
        ros::NodeHandle nh("~");
        mPubUSSideLeft = nh.advertise<sensor_msgs::Range>("/aadc/visualization/ultrasonic_side_left", 1);
        mPubUSSideRight = nh.advertise<sensor_msgs::Range>("/aadc/visualization/ultrasonic_side_right", 1);
        mPubUSRearLeft = nh.advertise<sensor_msgs::Range>("/aadc/visualization/ultrasonic_rear_left", 1);
        mPubUSRearCenter = nh.advertise<sensor_msgs::Range>("/aadc/visualization/ultrasonic_rear_center", 1);
        mPubUSRearRight = nh.advertise<sensor_msgs::Range>("/aadc/visualization/ultrasonic_rear_right", 1);
        ros::Subscriber subUS = nh.subscribe("/aadc/ultrasonic", 1, &UltrasonicVisualizationNode::callbackUS, this);

        m_broadcaster = new tf::TransformBroadcaster();


        mMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        mMsg.field_of_view = 0.3 * 30 * M_PI / 180.f; // 30deg according to audi, but reduced for better rviz results
        mMsg.min_range = 0.02;
        mMsg.max_range = 4.0;


        ros::Rate loop_rate(10);

        // Init controller
        while (ros::ok()) {            
            // Ros lifecycle
            ros::spinOnce();

            loop_rate.sleep();
        }

        return 0;
    }

    void callbackUS(const ros_oadrive::Ultrasonic msg)
    {
      transmit(mPubUSSideLeft, "ultrasonic_sl_frame", 0.25, 0.15, M_PI / 2, msg.header.stamp, msg.sideLeft);
      transmit(mPubUSSideRight, "ultrasonic_sr_frame", 0.25, -0.15, -M_PI / 2, msg.header.stamp, msg.sideRight);
      transmit(mPubUSRearLeft, "ultrasonic_rl_frame", -0.1, 0.1, 0.8 * M_PI, msg.header.stamp, msg.rearLeft);
      transmit(mPubUSRearCenter, "ultrasonic_rc_frame", -0.12, 0, M_PI, msg.header.stamp, msg.rearCenter);
      transmit(mPubUSRearRight, "ultrasonic_rr_frame", -0.1, -0.1, -0.8 * M_PI, msg.header.stamp, msg.rearRight);
    }

    void transmit(ros::Publisher &pub, std::string frame, float dx, float dy, float yaw, ros::Time time, float range) {
      tf::Quaternion orientation = tf::Quaternion(0, 0, 0, 1);
      orientation.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), yaw);
      m_broadcaster->sendTransform(
          tf::StampedTransform(tf::Transform(orientation, tf::Vector3(dx, dy, 0.f)),
          ros::Time::now(),
          "oriented_car",
          frame));

    //   if (range >= 399) return; // Skip bugging values

      if (range >= 399) {
          range = 0;
      }

      mMsg.header = std_msgs::Header();
      mMsg.header.stamp = time;
      mMsg.header.frame_id = frame;

      mMsg.range = range / 100.f; // cm to m
      
      pub.publish(mMsg);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "UltrasonicVisualizationNode");
    UltrasonicVisualizationNode rosNode;
    return rosNode.run();
}
