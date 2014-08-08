#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
import rospy
from std_msgs.msg import String

rospy.init_node("hi_alex")
pub = rospy.Publisher("porn_drugs_sex", String)
pub2 = rospy.Publisher("zzzz_crazy_mandeep", String)

while not rospy.is_shutdown():
    pub.publish("hahaha")
    pub2.publish("i am trolling")