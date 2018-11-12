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
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------


#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <ros_oadrive/EvokeTransition.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "StateMachineTestNode");
  ros::NodeHandle n;

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    ros::ServiceClient client = n.serviceClient<ros_oadrive::EvokeTransition>("/aadc/statemachine/evoke_transition");

    std::cin.get();
    ros_oadrive::EvokeTransition srv;
    srv.request.event.type = "RESET_COMMAND";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "START_COMMAND";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "STOP_COMMAND";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "START_COMMAND";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "APPROACH_INTERSECTION";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "TURN_AT_INTERSECTION";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "LEAVE_INTERSECTION";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "APPROACH_INTERSECTION";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "RESET_COMMAND";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "PREPARE_PULLING_OUT";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "START_PULLING_OUT";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);
    std::cin.get();

    srv.request.event.type = "FINISH_PULLING_OUT";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);

    std::cin.get();
    srv.request.event.type = "STOP_COMMAND";
    ROS_INFO_STREAM("Calling server with " + srv.request.event.type);
    client.call(srv);
  }

  return 0;
} 
