#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from .entity import Entity
from hri_msgs.msg import EntityMsg


class Saliency(Entity):
    ENTITY_TYPE = 'saliency'

    def __init__(self, saliency_number):
        Entity.__init__(self, Saliency.ENTITY_TYPE, Saliency.ENTITY_TYPE + str(saliency_number), None)

    @classmethod
    def make(cls, entity_num):
        return Saliency(entity_num)

    def default_tf_frame_id(self):
        return self.tf_frame_id()



