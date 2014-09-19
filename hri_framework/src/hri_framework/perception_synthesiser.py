#!/usr/bin/env python
import rospy
from hri_msgs.srv import AddEntity
from threading import Thread
import tf


class EntitySource():

    def __init__(self, topic_name, topic_type, entity_type):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.entity_type = entity_type

class SingleEntitySource():

    def __init__(self, topic_name, topic_type, entity_type):
        EntitySource.__init__(topic_name, topic_type, entity_type)
        self.data = None
        self.initialized = False
        self.sub = rospy.Subscriber(topic_name, topic_type, self.data_callback)
        self.br = tf.TransformBroadcaster()
        self.add_entity_srv = rospy.ServiceProxy('add_entity_service', AddEntity)

    def data_callback(self, msg):
        self.data = msg

    def update(self):
        if self.data is not None:
            if not self.initialized:
                self.add_entity_srv(self.entity_type, 1)
                self.initialized = True

            time = self.data.header.stamp
            frame_id = self.data.header.frame_id
            self.br.sendTransform(self.data.point, (0, 0, 0, 1), time, frame_id, self.entity_type)



class PerceptionSynthesizer(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.entity_sources = []
        self.rate = rospy.Rate(10)

    def add_entity_source(self, entity_source):
        self.entity_sources.append(entity_source)

    def run(self):
        while not rospy.is_shutdown():
            for source in self.entity_sources:
                source.update()

            self.rate.sleep()





