#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
import rospy
from .entity import Entity


class Person(Entity):
    def __init__(self, entity_id):
        Entity.__init__(self, 'person', entity_id=entity_id)
        self.head = Head(entity_id)
        self.left_hand = LeftHand(entity_id)
        self.right_hand = RightHand(entity_id)

    def say_to_gaze_tf_id(self):
        return self.head.base_link()

    def say_to_gesture_tf_id(self):
        return self.base_link()

    def base_link(self):
        return "torso_" + str(self.entity_id)


class Head(Entity):
    def __init__(self, entity_id):
        Entity.__init__(self, 'head', entity_id=entity_id)

    def base_link(self):
        return self.entity_type + "_" + str(self.entity_id)


class LeftHand(Entity):
    def __init__(self, entity_id):
        Entity.__init__(self, 'left_hand', entity_id=entity_id)

    def base_link(self):
        return self.entity_type + "_" + str(self.entity_id)


class RightHand(Entity):
    def __init__(self, entity_id):
        Entity.__init__(self, 'right_hand', entity_id=entity_id)

    def base_link(self):
        return self.entity_type + "_" + str(self.entity_id)