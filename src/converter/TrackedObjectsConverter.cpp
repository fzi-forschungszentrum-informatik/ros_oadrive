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
 *
 * \author  Kolja Esders <esders@fzi.de>
 * \date    2017
 *
 */
//----------------------------------------------------------------------
#include "oadrive_obstacle/ObjectTracker.h"
#include "ros_oadrive/TrackedObjectConverter.h"
#include "ros_oadrive/TrackedObjects.h"

#include "ros_oadrive/TrackedObjectsConverter.h"
#include "ros_oadrive/TrafficSignConverter.h"

namespace ros_oadrive {

TrackedObjects TrackedObjectsConverter::toMessage(
    const oadrive::obstacle::TrackedObjects &objects) {
  ros_oadrive::TrackedObjects objects_msg;

  for (auto &car : objects.cars) {
    ros_oadrive::TrackedCar car_msg;
    car_msg.object = TrackedObjectConverter::toMessage(car.object);
    car_msg.speed.x = car.speed.getX();
    car_msg.speed.y = car.speed.getY();
    objects_msg.cars.push_back(car_msg);
  }
  for (auto &person : objects.persons) {
    ros_oadrive::TrackedPerson person_msg;
    person_msg.object = TrackedObjectConverter::toMessage(person.object);
    person_msg.speed.x = person.speed.getX();
    person_msg.speed.y = person.speed.getY();
    person_msg.height = person.height;
    person_msg.heightErrorCovariance = person.heightErrorCovariance;
    objects_msg.persons.push_back(person_msg);
  }
  for(auto &obstacle: objects.obstacles)
  {
    ros_oadrive::TrackedObject obstacle_msg = TrackedObjectConverter::toMessage(obstacle);
    objects_msg.obstacles.push_back(obstacle_msg);
  }
  for (auto &trafficSign: objects.trafficSigns) {
    ros_oadrive::TrafficSign trafficSign_msg = TrafficSignConverter::toMessage(trafficSign);
    objects_msg.trafficSigns.push_back(trafficSign_msg);
  }
  objects_msg.follow_car_id = objects.follow_car_id;
  return objects_msg;
}

oadrive::obstacle::TrackedObjects TrackedObjectsConverter::fromMessage(
    const TrackedObjects &objects_msg) {
  oadrive::obstacle::TrackedObjects objects;

  for (auto &car_msg : objects_msg.cars) {
    oadrive::obstacle::TrackedCar car;
    car.object = TrackedObjectConverter::fromMessage(car_msg.object);
    car.speed.setX(car_msg.speed.x);
    car.speed.setY(car_msg.speed.y);
    objects.cars.push_back(car);
  }
  for (auto &person_msg : objects_msg.persons) {
    oadrive::obstacle::TrackedPerson person;
    person.object = TrackedObjectConverter::fromMessage(person_msg.object);
    person.speed.setX(person_msg.speed.x);
    person.speed.setY(person_msg.speed.y);
    person.height = person_msg.height;
    person.heightErrorCovariance = person.heightErrorCovariance;
    objects.persons.push_back(person);
  }
  for (auto &obstacle_msg : objects_msg.obstacles)
  {
    oadrive::obstacle::TrackedObject obstacle = TrackedObjectConverter::fromMessage(obstacle_msg);
    objects.obstacles.push_back(obstacle);
  }
  for (auto &trafficSign_msg : objects_msg.trafficSigns) {
    oadrive::world::TrafficSign trafficSign = TrafficSignConverter::fromMessage(trafficSign_msg);
    objects.trafficSigns.push_back(trafficSign);
  }
  objects.follow_car_id = objects_msg.follow_car_id;
  return objects;
}
}  // namespace ros_oadrive
