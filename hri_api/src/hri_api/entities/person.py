#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
import rospy
from .entity import Entity
from hri_api.entities import World

'person_1_head'
'person_2_left_hand'


class Person(Entity):
    def __init__(self, entity_type, entity_id):
        Entity.__init__(self, entity_type, entity_id)
        self.head = Head('head', self)
        self.torso = Torso('torso', self)
        self.left_hand = LeftHand('left_hand', self)
        self.right_hand = RightHand('right_hand', self)
        World().add_create_entity_callback(Person.create_person)

    @staticmethod
    def create_person(entity_msg):
        return Person(entity_msg.type, entity_msg.type + '_' + entity_msg.number)

    def say_to_gaze_tf_id(self):
        return self.head.base_link()

    def say_to_gesture_tf_id(self):
        return self.head.base_link()

    def base_link(self):
        return self.torso


class Head(Entity):
    def __init__(self, entity_type, parent):
        Entity.__init__(self, self.entity_type)

    def base_link(self):
        return self.parent.base_link() + '_' + selfen


class LeftHand(Entity):
    def __init__(self, parent):
        Entity.__init__(self, 'left_hand', entity_id=entity_id)

    def base_link(self):
        return self.entity_type + "_" + str(self.entity_id)


class RightHand(Entity):
    def __init__(self, parent):
        Entity.__init__(self, 'right_hand', entity_id=entity_id)

    def base_link(self):
        return self.entity_type + "_" + str(self.entity_id)