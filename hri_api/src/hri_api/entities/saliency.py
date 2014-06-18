#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from .entity import Entity
from hri_msgs.msg import EntityMsg


class Saliency(Entity):
    def __init__(self, entity_type, entity_id):
        Entity.__init__(self, entity_type, entity_id, None)

    @staticmethod
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



