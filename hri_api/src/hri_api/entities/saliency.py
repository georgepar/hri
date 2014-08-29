#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from .entity import Entity
from hri_msgs.msg import EntityMsg


class Saliency(Entity):
    ENTITY_TYPE = 'saliency'

    def __init__(self, saliency_number):
        Entity.__init__(self, Saliency.ENTITY_TYPE, Saliency.ENTITY_TYPE + str(saliency_number), None)

    @staticmethod #TODO: make a create entity service in World, for objects that arrive from perception
    def create_saliency(entity_msg):
        if not isinstance(entity_msg, EntityMsg):
            raise TypeError("create_saliency() parameter entity_msg={0} is not a subclass of EntityMsg".format(entity_msg))

        return Saliency(entity_msg.type, entity_msg.type + '_' + str(entity_msg.number))

    def say_to_gaze_tf_id(self):
        return self.head.base_link()

    def say_to_gesture_tf_id(self):
        return self.base_link()

    def default_tf_frame_id(self):
        return self.tf_frame_id()



