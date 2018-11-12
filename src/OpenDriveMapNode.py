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


import sys, rospy, os
from std_msgs.msg import String
#sys.path.append('../../../../ic_workspace/packages/oadrive/script/')

CONVERTER_SCRIPT = os.path.dirname(__file__) + "/../../../../ic_workspace/packages/oadrive/script/convert_odr.py"


from subprocess import call, Popen
import os

class OpenDriveMapNode():
    def __init__(self):
        rospy.init_node('OpenDriveMapNode')

        self.odr_sub = rospy.Subscriber('/aadc/opendrive_map', String, self.opendrive_callback)
        self.map_pub = rospy.Publisher('/aadc/alpaka_map', String, queue_size=10)

        self.map_server = None

    # spin up a map server publishing the map in rviz
    def run_map_server(self, path):
        if self.map_server is not None:
            self.map_server.kill()

        self.map_server = Popen(["rosrun", "map_server", "map_server", path, "_frame_id:=world"])
        
    
    def opendrive_callback(self, msg):
        TMP_INPUT_FILE = "/tmp/received.xodr"
        TMP_OUTPUT = "/tmp/converted"

        if len(msg.data) > 0:
            # unforuntely ros does not support python3 directly and our converter script is python3
            # so we have to spin an extra python3 process for the conversion

            with open(TMP_INPUT_FILE, "w") as xodr:
                xodr.write(msg.data)

            # TMP_INPUT_FILE = "/home/simon/Desktop/oadrive/script/aadc2018#1.xodr"

            retcode = call([CONVERTER_SCRIPT, TMP_INPUT_FILE, TMP_OUTPUT])

            if retcode == 0:
                with open(TMP_OUTPUT + ".xml") as xml:
                    new_msg = String()
                    new_msg.data = xml.read()

                    self.map_pub.publish(new_msg)

                    self.run_map_server(TMP_OUTPUT + ".yml")


if __name__ == "__main__":
    node = OpenDriveMapNode()
    rospy.spin()