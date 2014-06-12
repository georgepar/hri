#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')

import math
from geometry_msgs.msg import Point

class GeomMath(object):
    @staticmethod
    def distance_between(p1, p2):
        return math.sqrt(p1.x*p2.x + p1.y*p2.y + p1.z*p2.z)

    @staticmethod
    def is_infront_of(p1, p2):
        if p1.x >= p2.x:
            return True
        return False

    @staticmethod
    def is_behind(p1, p2):
        if p1.x < p2.x:
            return True
        return False

    @staticmethod
    def is_left_of(p1, p2):
        if p1.y >= p2.y:
            return True
        return False

    @staticmethod
    def is_right_of(p1, p2):
        if p1.y < p2.y:
            return True
        return False
