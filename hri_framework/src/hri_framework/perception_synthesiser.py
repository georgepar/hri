#!/usr/bin/env python
import rospy
from hri_msgs.srv import AddEntity
from threading import Thread
import tf
import importlib
import yaml
from threading import RLock
from hri_msgs.srv import SetVisibility


class HierarchicalEntity():

    def __init__(self, entity_type, pose_attr=None, parent=None):
        self.entity_type = entity_type
        self.pose_attr = pose_attr
        self.br = tf.TransformBroadcaster()
        self.tl = tf.TransformListener()
        self.parent = parent
        self.global_id = None
        self.local_id = None
        self.map_frame = None,

        self.visible = True
        self.last_time = None
        self.pose_stamped = None
        self.children = []

    def get_root(self):
        if self.parent is None:
            return self
        else:
            self.get_root()

    def add_child(self, child):
        self.children.append(child)

    def frame_id(self):
        if self.parent is not None:
            root = self.get_root()
            return root.entity_type + root.local_id + self.entity_type
        else:
            raise Exception('HierarchicalEntity root doesnt have frame_id')

    def update_data(self, msg):
        if self.parent is not None:
            self.pose_stamped = getattr(msg, self.pose_attr)

        for entity in self.children:
            entity.update_data(msg)

    def publish_transform(self):

        if self.parent is not None:
            root = self.get_root()

            if root.visible:
                time = self.pose_stamped.header.stamp
                parent_frame = self.pose_stamped.header.frame_id
                frame_id = self.frame_id()
                pose = self.pose_stamped.pose
                self.br.sendTransform(pose.position, pose.orientation, time, parent_frame, frame_id)
                root.last_time = time
            else:
                frame_id = self.frame_id()
                (trans, rot) = self.tl.lookupTransform(root.map_frame, frame_id, root.last_time)
                self.br.sendTransform(trans, rot, rospy.Time.now(), root.map_frame, frame_id)

        for child in self.children:
            child.publish_transform()


class HierarchicalEntityListSource():

    def __init__(self, topic_name, module_name, class_name, attribute_name, entity_type, local_id_attr):
        self.topic_name = topic_name
        self.module_name = module_name
        self.class_name = class_name
        self.attribute_name = attribute_name
        self.entity_type = entity_type
        self.local_id_attr = local_id_attr
        self.entities = []
        self.lock = RLock()
        self.children_definitions = []
        self.add_entity_srv = rospy.ServiceProxy('add_entity', AddEntity)
        self.set_visibility_srv = rospy.ServiceProxy('set_visibility', SetVisibility)

        module = importlib.import_module(self.module_name)
        data_class = getattr(module, self.class_name)
        self.source_sub = rospy.Subscriber(self.topic_name, data_class, self.update_entities, queue_size=10)

    def update_entities(self, msg):
        rospy.loginfo('hello: ' + str(msg))

        with self.lock:
            raw_list = getattr(msg, self.attribute_name)
            visible_entities = []

            for raw_data in raw_list:
                local_id = getattr(raw_data, self.local_id_attr)

                # If entity hasn't been seen yet, then create it
                if not self.does_entity_exist(local_id):
                    entity = HierarchicalEntity(self.entity_type)
                    entity.local_id = local_id
                    entity.map_frame = 'map'

                    for (child_type, pose_attr) in self.children_definitions:
                        child = HierarchicalEntity(child_type, pose_attr, entity)
                        entity.add_child(child)

                    self.add_entity(entity)
                    entity.update_data(msg)
                    visible_entities.append(entity)
                else:
                    entity = self.get_entity(local_id)

                    if not entity.visible:
                        entity.visible = True
                        self.set_visibility_srv(entity.global_id, True)

                    visible_entities.append(entity)

            invisible_entities = list(set(self.entities) - set(visible_entities))

            for entity in invisible_entities:
                if entity.visible:
                    entity.visible = False
                    self.set_visibility_srv(entity.global_id, False)

    def publish_transforms(self):
        with self.lock:
            for entity in self.entities:
                entity.publish_transform()

    def add_child_definition(self, child_type, pose_attr):
        self.children_definitions.append((child_type, pose_attr))

    def add_entity(self, entity):
        res = self.add_entity_srv(self.entity_type, entity.local_id)
        entity.global_id = res.global_id
        self.entities.append(entity)

    def get_entity(self, local_id):
        for entity in self.entities:
            if entity.local_id == local_id:
                return entity

    def does_entity_exist(self, local_id):
        for entity in self.entities:
            if entity.local_id == local_id:
                return True
        return False


class PerceptionSynthesizer(Thread):

    def __init__(self):
        super(PerceptionSynthesizer, self).__init__()

        if rospy.has_param('~perception_sources_yaml'):
            path = rospy.get_param('~perception_sources_yaml')
        else:
            raise Exception('Please specify perception_sources_yaml parameter')

        self.rate = rospy.Rate(10)
        self.sources = PerceptionSynthesizer.parse_yaml(path)
        rospy.loginfo(str(self.sources))

    @staticmethod
    def parse_yaml(path):
        with open(path, 'r') as file:
            config = yaml.load(file)

        sources = []

        for source_name, attributes in list(config.items()):
            src_type = attributes['source_type']
            module = importlib.import_module('hri_framework')
            source_class = getattr(module, src_type)

            if type(source_class) is type(HierarchicalEntityListSource):
                topic_name = attributes['topic']
                module_name = attributes['module']
                class_name = attributes['class']
                attribute_name = attributes['attribute']
                entity_type = attributes['entity_type']
                local_id_attr = attributes['local_id_attr']

                source = HierarchicalEntityListSource(topic_name, module_name, class_name, attribute_name, entity_type, local_id_attr)

                for child in attributes['children']:
                    child_type = child['type']
                    pose_attr = child['pose_attr']
                    source.add_child_definition(child_type, pose_attr)

                sources.append(source)

        return sources

    def run(self):
        while not rospy.is_shutdown():
            for source in self.sources:
                source.publish_transforms()

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('perception_synthesiser')
    ps = PerceptionSynthesizer()
    rospy.loginfo('PerceptionSynthesizer starting...')
    ps.start()
    rospy.loginfo('PerceptionSynthesizer started...')
    rospy.spin()



