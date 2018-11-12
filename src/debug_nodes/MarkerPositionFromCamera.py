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

import rospy

from ros_oadrive.msg import MarkerPosition, ImagePosition

rospy.init_node('MarkerPositionFromCamera')
pub = rospy.Publisher('/aadc/marker_position', MarkerPosition, queue_size=10)

def callback(image_position):
    msg = MarkerPosition()

    msg.pose = image_position.pose
    msg.speed = 0
    msg.radius = 0

    pub.publish(msg)

rospy.Subscriber("/aadc/front/birdview", ImagePosition, callback)
rospy.spin()