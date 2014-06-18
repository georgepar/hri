#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from .entity import Entity
from hri_msgs.msg import EntityMsg


#World().add_create_entity_callback(Person.create_person)
class Person(Entity):
    def __init__(self, entity_type, entity_id):
        Entity.__init__(self, entity_type, entity_id, None)
        head = 'head'
        neck = 'neck'
        left_shoulder = 'left_shoulder'
        left_elbow = 'left_elbow'
        left_hand = 'left_hand'
        right_shoulder = 'right_shoulder'
        right_elbow = 'right_elbow'
        right_hand = 'right_hand'
        torso = 'torso'

        self.head = Head(head, head, self)
        self.neck = Neck(neck, neck, self)
        self.torso = Torso(torso, torso, self)
        self.left_hand = LeftHand(left_hand, left_hand, self)
        self.right_hand = RightHand(right_hand, right_hand, self)

    @staticmethod
    def create_person(entity_msg):
        if not isinstance(entity_msg, EntityMsg):
            raise TypeError("create_person() parameter entity_msg={0} is not a subclass of EntityMsg".format(entity_msg))

        return Person(entity_msg.type, entity_msg.type + '_' + str(entity_msg.number))

    def say_to_gaze_tf_id(self):
        return self.head.tf_frame_id()

    def say_to_gesture_tf_id(self):
        return self.head.tf_frame_id()

    def default_tf_frame_id(self):
        return self.torso.tf_frame_id()


class Head(Entity):
    def __init__(self, entity_type, entity_id, parent):
        Entity.__init__(self, entity_type, entity_id, parent)


class Neck(Entity):
    def __init__(self, entity_type, entity_id, parent):
        Entity.__init__(self, entity_type, entity_id, parent)


class Torso(Entity):
    def __init__(self, entity_type, entity_id, parent):
        Entity.__init__(self, entity_type, entity_id, parent)


class LeftHand(Entity):
    def __init__(self, entity_type, entity_id, parent):
        Entity.__init__(self, entity_type, entity_id, parent)


class RightHand(Entity):
    def __init__(self, entity_type, entity_id, parent):
        Entity.__init__(self, entity_type, entity_id, parent)
