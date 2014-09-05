#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from .entity import Entity
from hri_msgs.msg import EntityMsg


#World().add_create_entity_callback(Person.create_person)
class Person(Entity):
    ENTITY_TYPE = 'person'

    def __init__(self, person_number):
        Entity.__init__(self, Person.ENTITY_TYPE, Person.ENTITY_TYPE + str(person_number), None)
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

    @classmethod
    def make(cls, entity_num):
        return Person(entity_num)

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
