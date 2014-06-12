#!/usr/bin/env python
import roslib

roslib.load_manifest('hri_framework')
import rospy

class RobotParams(object):
    def __init__(self):
        self.id_name = "~robot_id"

    def robot_id(self):
        if not rospy.has_param(self.id_name):
            raise Exception("Please specify a robot_id in the launch file.")
        return rospy.get_param(self.id_name)