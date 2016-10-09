#!/usr/bin/env python
import rospy
from hri_msgs.srv import AddEntity
import yaml
from threading import RLock
from hri_msgs.srv import SetVisibility
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import UInt16MultiArray
from rospy import ServiceException


class PerceivedEntity():

    def __init__(self, local_id):
        self.local_id = local_id
        self.global_id = None
        self.visible = True

    def set_global_id(self, global_id):
        self.global_id = global_id

    def set_visibility(self, visible):
        self.visible = visible


class PerceptionSource():

    def __init__(self, topic_name, entity_module, entity_class):
        self.topic_name = topic_name
        self.entity_module = entity_module
        self.entity_class = entity_class
        self.entity_lookup = {}
        self.entities = []
        self.lock = RLock()
        self.source_sub = None
        self.add_entity_srv = rospy.ServiceProxy('add_entity', AddEntity)
        self.set_visibility_srv = rospy.ServiceProxy('set_visibility', SetVisibility)

    def reset(self):
        with self.lock:
            if self.source_sub is not None:
                self.source_sub.unregister()

            self.entity_lookup.clear()
            self.entities = []

    def enable(self):
        with self.lock:
            self.reset()
            self.source_sub = rospy.Subscriber(self.topic_name, UInt16MultiArray, self.update_entities, queue_size=10)

    def disable(self):
        with self.lock:
            self.reset()

    def update_entities(self, msg):
        local_id_list = msg.data

        with self.lock:
            visible = []

            for local_id in local_id_list:

                # If entity hasn't been seen yet, then create it
                if not self.exists(local_id):
                    entity = PerceivedEntity(local_id)
                    self.add_entity(entity)
                    visible.append(entity)
                else:
                    entity = self.get_entity(local_id)

                    if not entity.visible:
                        self.set_visibility(entity, True)

                    visible.append(entity)

            invisible = list(set(self.entities) - set(visible))

            for entity in invisible:
                if entity.visible:
                    self.set_visibility(entity, False)

    def set_visibility(self, entity, visible):
        entity.set_visibility(visible)

        try:
            self.set_visibility_srv(visible)
        except:
            self.disable()

    def add_entity(self, entity):

        try:
            res = self.add_entity_srv(self.entity_module, self.entity_class, entity.local_id)
            entity.global_id = res.global_id
            self.entities.append(entity)
            self.entity_lookup[entity.local_id] = entity
        except:
            self.disable()

    def get_entity(self, local_id):
        return self.entity_lookup[local_id]

    def exists(self, local_id):
        return local_id in self.entity_lookup


class PerceptionSynthesizer():

    def __init__(self):
        if rospy.has_param('~perception_sources_yaml'):
            path = rospy.get_param('~perception_sources_yaml')
        else:
            msg = 'Please specify perception_sources_yaml parameter'
            rospy.logerr(msg)
            raise Exception(msg)

        self.sources = PerceptionSynthesizer.parse_yaml(path)
        rospy.loginfo(str(self.sources))

        self.lock = RLock()
        self.enabled = False
        self.enable_srv = rospy.Service('perception_synthesiser/enable', Empty, self.enable_callback)
        self.disable_srv = rospy.Service('perception_synthesiser/disable', Empty, self.disable_callback)

    def enable_callback(self, req):
        with self.lock:
            self.enabled = True

            for source in self.sources:
                source.enable()

        return EmptyResponse()

    def disable_callback(self, req):
        with self.lock:
            self.enabled = False

            for source in self.sources:
                source.disable()

        return EmptyResponse()

    @staticmethod
    def parse_yaml(path):
        with open(path, 'r') as file:
            config = yaml.load(file)

        sources = []

        for source_name, attributes in list(config.items()):
            topic_name = attributes['topic']
            entity_data = attributes['entity']
            entity_module = entity_data['module']
            entity_class = entity_data['class']

            source = PerceptionSource(topic_name, entity_module, entity_class)
            sources.append(source)

        return sources


if __name__ == '__main__':
    rospy.init_node('perception_synthesiser')
    ps = PerceptionSynthesizer()
    rospy.spin()



