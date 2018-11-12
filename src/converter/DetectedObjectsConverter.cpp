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
 *
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
 * \date    2017
 */
//----------------------------------------------------------------------

#include "ros_oadrive/DetectedObjects.h"
#include "oadrive_obstacle/ObjectDetector.h"

#include "ros_oadrive/DetectedObjectsConverter.h"

namespace ros_oadrive {

// ToDo: assign object specific variables like height
DetectedObjects DetectedObjectsConverter::toMessage(const oadrive::obstacle::DetectedObjects &objectList)
{
  ros_oadrive::DetectedObjects objectList_msg;
  for(auto & car : objectList.cars)
  {
    ros_oadrive::DetectedCar car_msg;
    car_msg.object.poseVariance.x = car.object.poseVariance.getX();
    car_msg.object.poseVariance.y = car.object.poseVariance.getY();
    car_msg.object.pose.x = car.object.pose.getX();
    car_msg.object.pose.y = car.object.pose.getY();
    car_msg.object.width = car.object.width;
    car_msg.object.widthVariance = car.object.widthVariance;
    objectList_msg.cars.push_back(car_msg);
  }
  for(auto & person : objectList.persons)
  {
    ros_oadrive::DetectedPerson person_msg;
    person_msg.object.poseVariance.x = person.object.poseVariance.getX();
    person_msg.object.poseVariance.y = person.object.poseVariance.getY();
    person_msg.object.pose.x = person.object.pose.getX();
    person_msg.object.pose.y = person.object.pose.getY();
    person_msg.object.width = person.object.width;
    person_msg.object.widthVariance = person.object.widthVariance;
    person_msg.height = person.height;
    person_msg.heightVariance = person.heightVariance;
    objectList_msg.persons.push_back(person_msg);
  }
  return objectList_msg;
}

oadrive::obstacle::DetectedObjects DetectedObjectsConverter::fromMessage(const DetectedObjects &DetectedObjects_msg)
{
  oadrive::obstacle::DetectedObjects DetectedObjects;
  for(auto & car_msg : DetectedObjects_msg.cars)
  {
    oadrive::obstacle::DetectedCar car;
    car.object.poseVariance.setX(car_msg.object.poseVariance.x);
    car.object.poseVariance.setY(car_msg.object.poseVariance.y);
    car.object.pose.setX(car_msg.object.pose.x);
    car.object.pose.setY(car_msg.object.pose.y);
    car.object.width = car_msg.object.width;
    car.object.widthVariance = car_msg.object.widthVariance;
    DetectedObjects.cars.push_back(car);
  }
  for(auto & person_msg : DetectedObjects_msg.persons)
  {
    oadrive::obstacle::DetectedPerson person;
    person.object.poseVariance.setX(person_msg.object.poseVariance.x);
    person.object.poseVariance.setY(person_msg.object.poseVariance.y);
    person.object.pose.setX(person_msg.object.pose.x);
    person.object.pose.setY(person_msg.object.pose.y);
    person.object.width = person_msg.object.width;
    person.object.widthVariance = person_msg.object.widthVariance;
    person.height = person_msg.height;
    person.heightVariance = person_msg.heightVariance;
    DetectedObjects.persons.push_back(person);
  }
  return DetectedObjects;
 }
}  // namespace ros_oadrive

