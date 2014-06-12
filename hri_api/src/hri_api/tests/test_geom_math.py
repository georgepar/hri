from unittest import TestCase

__author__ = 'Jamie Diprose'

from geometry_msgs.msg import Point
from hri_api.math import GeomMath
import math

class TestGeomMath(TestCase):
    def __init__(self, *args, **kwargs):
        super(TestGeomMath, self).__init__(*args, **kwargs)
        self.center = Point(1, 1, 0)
        self.infront = Point(2, 1, 0)
        self.behind = Point(0, 1, 0)
        self.left = Point(1, 2, 0)
        self.right = Point(1, 0, 0)

    def test_distance_to(self):
        distance = GeomMath.distance_between(self.center, self.infront)
        self.assertEqual(distance, math.sqrt(3))

    def test_is_infront_of(self):
        self.assertTrue(GeomMath.is_infront_of(self.infront, self.center))
        self.assertFalse(GeomMath.is_infront_of(self.behind, self.center))

    def test_is_behind(self):
        self.assertTrue(GeomMath.is_behind(self.behind, self.center))
        self.assertFalse(GeomMath.is_behind(self.infront, self.center))

    def test_is_left_of(self):
        self.assertTrue(GeomMath.is_left_of(self.left, self.center))
        self.assertFalse(GeomMath.is_left_of(self.right, self.center))

    def test_is_right_of(self):
        self.assertTrue(GeomMath.is_right_of(self.right, self.center))
        self.assertFalse(GeomMath.is_right_of(self.left, self.center))


