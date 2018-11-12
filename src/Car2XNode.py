#!/usr/bin/env python
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------
__author__ = "Simon Roesler <simon.roesler@student.kit.edu>"


import sys, rospy, zmq, tf, argparse, uuid, json, logging, time
#from zyre_pyzmq import Zyre as Pyre
from pyre import Pyre, zhelper 
from rospy_message_converter import message_converter
from ros_oadrive.msg import DetectedObjects, DetectedObject, DetectedCar, DetectedPerson, Event, StreetPatchOccupancy, MarkerPosition, ExternalMarkerPosition, NavObstruction
from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D
from Queue import Queue
import std_msgs

def quaternion_from_pose2d(pose2d):
    q = tf.transformations.quaternion_from_euler(0, 0, pose2d.theta)

    res = Quaternion()
    res.x = q[0]
    res.y = q[1]
    res.z = q[2]
    res.w = q[3]

    return res

def yaw_from_pose(pose):
    quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    return euler[2]
    

class Car2XNode():
    def __init__(self, name):
        self.name = name
        self.out_queue = Queue()

        # ros stuff
        rospy.init_node('Car2XNode', anonymous=True)
        self.tf = tf.TransformListener()
        
        self.nav_obstruction_pub = rospy.Publisher('/aadc/nav_obstructions', NavObstruction, queue_size=10)
        self.object_detection_pub = rospy.Publisher('/aadc/object_detection/detected_objects', DetectedObjects, queue_size=10)
        self.marker_position_pub = rospy.Publisher('/aadc/external_marker_position', ExternalMarkerPosition, queue_size=20)
        self.obstruction_pub = rospy.Publisher('/aadc/planning/patch_occupancy', StreetPatchOccupancy, queue_size=20)
        
        self.jury_pub = rospy.Publisher('/aadc/jury/event', std_msgs.msg.String, queue_size=20)

        print("Waiting for tf..")
        self.tf.waitForTransform("world", "local", rospy.Time(), rospy.Duration(1000000.0))
        self.tf.waitForTransform("local", "world", rospy.Time(), rospy.Duration(1000000.0))
        print("..DONE")

        # communication things
        self.stopped = False
        self.ctx = zmq.Context()
        self.outgoing = zhelper.zthread_fork(self.ctx, self.network_thread)

        print("Connecting..")
        # wait for ready from the thread
        try:
            time.sleep(1.0)
            self.outgoing.recv()
        except zmq.error.Again:
            rospy.logerr("ERROR! Again #0")
        print("..CONNECTED!")

        self.event_sub = rospy.Subscriber('/aadc/planning/event', Event, self.process_event)
        self.marker_position_sub = rospy.Subscriber('/aadc/marker_position', MarkerPosition, self.process_internal_marker_position)

        # ADD_TOPIC_PART_HERE
        self.object_detection_sub = rospy.Subscriber('/aadc/object_detection/detected_objects', DetectedObjects, self.process_internal_detected_objects)
        self.nav_obstruction_sub = rospy.Subscriber('/aadc/nav_obstructions', NavObstruction, self.process_internal_nav_obs)
        
        # Obstructions car2x disabled as the have no meaning anymore
        # We directly publish if a tile is obstructed now
        # self.obstruction_sub = rospy.Subscriber('/aadc/planning/patch_occupancy', StreetPatchOccupancy, self.process_internal_obstructions)

    
    def stop(self):
        self.stopped = True
        self.outgoing.send("$$STOP".encode('utf_8'))
    
    def network_thread(self, ctx, pipe):
        print("Network thread started..")
        n = Pyre(self.name)
        n.join("Car2X")
        n.start()

        poller = zmq.Poller()
        poller.register(pipe, zmq.POLLIN)
        poller.register(n.inbox, zmq.POLLIN)


        # wait for others
        while not n.peer_groups():
            time.sleep(0.0001)
            pass

        pipe.send('$$ready'.encode('utf-8'))

        while not self.stopped:
            items = dict(poller.poll())
            
            if pipe in items and items[pipe] == zmq.POLLIN:
                # handle outgoing messages
                try:
                    message = pipe.recv()
                    if message.decode('utf-8') == "$$STOP":
                        break
                    n.shouts("Car2X", message.decode('utf-8'))
                except zmq.error.Again:
                    rospy.logerr("ERROR! Again #1")
            if n.inbox in items and items[n.inbox] == zmq.POLLIN:
                try:
                    cmds = n.recv()

                    msg_type = cmds.pop(0)
                    msg_uuid = cmds.pop(0)
                    msg_name = cmds.pop(0)
                    if msg_type == "JOIN":
                        rospy.loginfo("{} joined!".format(msg_name))
                    elif msg_type == "EXIT":
                        rospy.loginfo("{} left!".format(msg_name))
                    elif msg_type == "SHOUT":
                        # handling incoming information
                        msg_channel = cmds.pop(0)
                        msg_str = cmds.pop(0)
                        self.process_data(msg_str, msg_name)
                except zmq.error.Again:
                    rospy.logerr("ERROR! Again #2")
        n.stop()
    

    def process_data(self, msg_str, host):
        if rospy.is_shutdown():
            print("ERROR; ROS SHUTDOWN!")
            return
        data = json.loads(msg_str)

        if not ("payload" in data and "topic" in data):
            rospy.logwarn("Dropping message with unknown format %s %s", msg_str, data)
            return

        if data["topic"] == "OBJECT_DETECTOR":
            self.process_external_detected_objects(data["payload"])
        elif data["topic"] == "OBSTRUCTIONS":
            self.process_external_obstructions(data["payload"])
        elif data["topic"] == "EGO_POSE":
            self.process_external_marker_position(data["payload"], host)
        elif data["topic"] == "NAV_OBS":
            self.process_external_nav_obs(data["payload"])
        elif data["topic"] == "GET_READY_REQ":
            self.process_get_ready(data["payload"])
        elif data["topic"] == "START_REQ":
            self.process_start(data["payload"])
        elif data["topic"] == "STOP_REQ":
            self.process_stop(data["payload"])
        else:
            rospy.logwarn("Received unknown data topic %s", data["topic"])

    def process_external_detected_objects(self, data):
        # 1. Put into ros msg
        msg = message_converter.convert_dictionary_to_ros_message('ros_oadrive/DetectedObjects', data)
        msg.external_observations = True # this is very important to not republish the observations to the car2x server


        # 2. Translate data from world back to our local frame
        # 2.1. Cars
        for x in msg.cars:
            x.object.pose = self.transform_pose2d(x.object.pose, source_frame="world", target_frame="local")
        # 2.2. Persons
        for x in msg.persons:
            x.object.pose = self.transform_pose2d(x.object.pose, source_frame="world", target_frame="local")

        # 3. Publish data to ROS on object detection topic
        self.object_detection_pub.publish(msg)

    # ADD_TOPIC_PART_HERE
    def process_internal_detected_objects(self, msg):
        if msg.external_observations:
            # we only want to handle own detections here
            return
        if len(msg.cars) == 0 and len(msg.persons) == 0:
            # only publish relevant things
            return

        # 1. translate poses from local to world
        # 1.1. Cars
        for x in msg.cars:
            x.object.pose = self.transform_pose2d(x.object.pose, source_frame="local", target_frame="world")
        # 1.2. Persons
        for x in msg.persons:
            x.object.pose = self.transform_pose2d(x.object.pose, source_frame="local", target_frame="world")

        data = message_converter.convert_ros_message_to_dictionary(msg)

        # 2. transmit to our "car2x" server
        try:
            self.outgoing.send(json.dumps({"topic": "OBJECT_DETECTOR", "payload": data}))
        except zmq.error.Again:
            rospy.logerr("ERROR EAGAIN!")

    def process_external_obstructions(self, data):
        # 1. Put into ros msg
        msg = message_converter.convert_dictionary_to_ros_message('ros_oadrive/StreetPatchOccupancy', data)
        msg.external_observations = True # this is very important to not republish the observations to the car2x server


        # 2. Translate data from world back to our local frame
        world_pose = self.transform_pose2d(Pose2D(x = msg.x, y = msg.y), source_frame="world", target_frame="local")
        msg.x = world_pose.x
        msg.y = world_pose.y

        # 3. Publish data to ROS on object detection topic
        self.obstruction_pub.publish(msg)

    def process_internal_nav_obs(self, msg):
        if msg.external_observations:
            # we only want to handle own detections here
            return

        data = message_converter.convert_ros_message_to_dictionary(msg)

        try:
            self.outgoing.send(json.dumps({"topic": "NAV_OBS", "payload": data}))
        except zmq.error.Again:
            rospy.logerr("ERROR EAGAIN!")

    def process_external_nav_obs(self, data):
        # 1. Put into ros msg
        msg = message_converter.convert_dictionary_to_ros_message('ros_oadrive/NavObstruction', data)
        msg.external_observations = True # this is very important to not republish the observations to the car2x server

        self.nav_obstruction_pub.publish(msg)

    def process_internal_obstructions(self, msg):
        if msg.external_observations:
            # we only want to handle own detections here
            return

        # 1. translate poses from local to world
        world_pose = self.transform_pose2d(Pose2D(x = msg.x, y = msg.y), source_frame="local", target_frame="world")
        
        msg.x = world_pose.x
        msg.y = world_pose.y

        data = message_converter.convert_ros_message_to_dictionary(msg)

        # 2. transmit to our "car2x" server
        try:
            self.outgoing.send(json.dumps({"topic": "OBSTRUCTIONS", "payload": data}))
        except zmq.error.Again:
            rospy.logerr("ERROR EAGAIN!")
    

    def process_event(self, data):
        if data.type == "START_TELE_OP_EVENT":
            try:
                self.outgoing.send(json.dumps({"topic": "TELE_OP_REQ", "payload": self.name}))
            except zmq.error.Again:
                rospy.logerr("ERROR EAGAIN!")
        if data.type == "END_TELE_OP_EVENT":
            try:
                self.outgoing.send(json.dumps({"topic": "END_TELE_OP_REQ", "payload": self.name}))
            except zmq.error.Again:
                rospy.logerr("ERROR EAGAIN!")

    def process_internal_marker_position(self, msg):
        world_pose = self.transform_pose2d(msg.pose, source_frame="local", target_frame="world")
        
        msg.pose.x = world_pose.x
        msg.pose.y = world_pose.y
        msg.pose.theta = world_pose.theta

        data = message_converter.convert_ros_message_to_dictionary(msg)

        # 2. transmit to our "car2x" server
        try:
            self.outgoing.send(json.dumps({"topic": "EGO_POSE", "payload": data}))
        except zmq.error.Again:
            rospy.logerr("ERROR EAGAIN!")

    def process_external_marker_position(self, data, host): 
        # 1. Put into ros msg
        mp_msg = message_converter.convert_dictionary_to_ros_message('ros_oadrive/MarkerPosition', data)

        # 2. Translate data from world back to our local frame
        local_pose = self.transform_pose2d(mp_msg.pose, source_frame="world", target_frame="local")
        mp_msg.pose.x = local_pose.x
        mp_msg.pose.y = local_pose.y
        mp_msg.pose.theta = local_pose.theta
        
        msg = ExternalMarkerPosition()
        msg.host = host
        msg.marker_position = mp_msg
        # 3. Publish data to ROS on ext marker pos topic
        self.marker_position_pub.publish(msg)

        # also publish pose to the object tracker so we can give way without ever detecting the car
        # (Because this may happen e.g. if we drive without the cover)
        det_object = DetectedObject()

        det_object.pose = mp_msg.pose

        # 20cm tolerance from audi..
        det_object.poseVariance.x = 0.04
        det_object.poseVariance.y = 0.04
        det_object.poseVariance.theta = 0.01

        # those values are just random, but they have no effect on the system anyway atm, since a car is a car, no matter how big
        det_object.width = 0.6
        det_object.widthVariance = 0


        det_car = DetectedCar(object=det_object)
        det_msg = DetectedObjects(cars=[det_car], external_observations=True)

        # 3. Publish data to ROS on ext marker pos topic
        self.object_detection_pub.publish(det_msg)

    def process_get_ready(self, host):
        if host == "ALL" or host == self.name:
            msg = std_msgs.msg.String(data="GET_READY_EVENT")
            self.jury_pub.publish(msg)

    def process_stop(self, host):
        if host == "ALL" or host == self.name:
            msg = std_msgs.msg.String(data="STOP_EVENT")
            self.jury_pub.publish(msg)

    def process_start(self, host):
        if host == "ALL" or host == self.name:
            msg = std_msgs.msg.String(data="START_EVENT")
            self.jury_pub.publish(msg)


    def build_detected_object(self, data):
        obj = DetectedObject()
        obj.pose.x = data.pose.x
        obj.pose.y = data.pose.y
        obj.pose.theta = data.pose.theta

        obj.poseVariance.x = data.poseVariance.x
        obj.poseVariance.y = data.poseVariance.y
        obj.poseVariance.theta = data.poseVariance.theta

        obj.width = data.width
        obj.widthVariance = data.widthVariance

        return obj


    def transform_pose2d(self, pose2d, source_frame, target_frame):
        #t = self.tf.getLatestCommonTime(target_frame, source_frame)

        # put pose into pose stamped
        source_pose = PoseStamped()
        source_pose.header.frame_id = source_frame
        source_pose.pose.position.x = pose2d.x
        source_pose.pose.position.y = pose2d.y

        source_pose.pose.orientation = quaternion_from_pose2d(pose2d)
        
        # and transform into world
        target_pose = self.tf.transformPose(target_frame, source_pose)
        
        # and back into a pose
        res = Pose2D()
        res.x = target_pose.pose.position.x
        res.y = target_pose.pose.position.y
        res.theta = yaw_from_pose(target_pose)

        return res

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', dest='name', default="Alpacanonymous")
    args = parser.parse_args(rospy.myargv()[1:])

    logger = logging.getLogger("pyre")
    logger.setLevel(logging.INFO)
    logger.addHandler(logging.StreamHandler())
    logger.propagate = False

    node = Car2XNode(args.name)
    rospy.spin()
    node.stop()