#!/usr/bin/env python
import rospy
from hri_msgs.srv import AddEntity
from threading import Thread
import tf
import importlib
import yaml
from threading import RLock
from hri_msgs.srv import SetVisibility
from std_srvs.srv import Empty, EmptyResponse
from tf import transformations


class HierarchicalEntity():

    def __init__(self, tf_frame, pose_attr=None, parent=None):
        self.tf_frame = tf_frame
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
            return self.parent.get_root()

    def add_child(self, child):
        self.children.append(child)

    def set_visible(self):
        self.visible = True

    def set_hidden(self):
        self.visible = False

    def frame_id(self):
        if self.parent is not None:
            root = self.get_root()
            return root.tf_frame + str(root.local_id) + '_' + self.tf_frame
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
                time = rospy.Time().now() #self.pose_stamped.header.stamp
                parent_frame = self.pose_stamped.header.frame_id
                frame_id = self.frame_id()
                pose = self.pose_stamped.pose
                self.br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                                      (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) ,
                                      time, frame_id, parent_frame)

                root.last_time = time - rospy.Duration(1)
            else:
                frame_id = self.frame_id()
                (trans, rot) = self.tl.lookupTransform(root.map_frame, frame_id, root.last_time)
                self.br.sendTransform(trans, rot, rospy.Time.now(), root.map_frame, frame_id)

        for child in self.children:
            child.publish_transform()


class HierarchicalEntityListSource():

    def __init__(self, topic_name, module_name, class_name, attribute_name, parent_module, parent_class, parent_tf_frame, local_id_attr):
        self.topic_name = topic_name
        self.module_name = module_name
        self.class_name = class_name
        self.attribute_name = attribute_name

        self.parent_module = parent_module
        self.parent_class = parent_class
        self.parent_tf_frame = parent_tf_frame
        self.local_id_attr = local_id_attr

        self.entities = []
        self.entity_lookup = {}
        self.lock = RLock()
        self.children_definitions = []
        self.add_entity_srv = rospy.ServiceProxy('add_entity', AddEntity)
        self.set_visibility_srv = rospy.ServiceProxy('set_visibility', SetVisibility)
        self.source_sub = None

    def enable(self):
        module = importlib.import_module(self.module_name)
        data_class = getattr(module, self.class_name)
        self.source_sub = rospy.Subscriber(self.topic_name, data_class, self.update_entities, queue_size=100)

    def disable(self):
        self.source_sub.unregister()
        self.entities = []

    def update_entities(self, msg):
        raw_list = getattr(msg, self.attribute_name)

        with self.lock:
            raw_list = getattr(msg, self.attribute_name)
            #visible_entities = []

            for raw_data in raw_list:
                local_id = getattr(raw_data, self.local_id_attr)

                if local_id in self.entity_lookup:
                    entity = self.get_entity(local_id)
                    entity.update_data(raw_data)
                    entity.publish_transform()
                else:
                    entity = HierarchicalEntity(self.parent_tf_frame)
                    entity.local_id = local_id
                    entity.map_frame = 'camera_depth_frame' #TODO: pull value from file

                    for (child_tf_frame, pose_attr) in self.children_definitions:
                        child = HierarchicalEntity(child_tf_frame, pose_attr, entity)
                        entity.add_child(child)

                    self.add_entity(entity)
                    entity.update_data(raw_data)

                # # If entity hasn't been seen yet, then create it
                # if not self.does_entity_exist(local_id):
                #     entity = HierarchicalEntity(self.parent_tf_frame)
                #     entity.local_id = local_id
                #     entity.map_frame = 'camera_depth_frame' #TODO: pull value from file
                #
                #     for (child_tf_frame, pose_attr) in self.children_definitions:
                #         child = HierarchicalEntity(child_tf_frame, pose_attr, entity)
                #         entity.add_child(child)
                #
                #     self.add_entity(entity)
                #     entity.update_data(raw_data)
                #     visible_entities.append(entity)
                # else:
                #     entity = self.get_entity(local_id)
                #     entity.update_data(raw_data)

                    # if not entity.visible:
                    #     entity.set_visible()
                    #     self.set_visibility_srv(entity.global_id, True)
                    #
                    # visible_entities.append(entity)

            # invisible_entities = list(set(self.entities) - set(visible_entities))
            #
            # for entity in invisible_entities:
            #     if entity.visible:
            #         entity.set_hidden()
            #         self.set_visibility_srv(entity.global_id, False)

    def publish_transforms(self):
        with self.lock:
            for entity in self.entities:
                entity.publish_transform()

    def add_child_definition(self, child_type, pose_attr):
        self.children_definitions.append((child_type, pose_attr))

    def add_entity(self, entity):
        res = self.add_entity_srv(self.parent_module, self.parent_class, entity.local_id)
        entity.global_id = res.global_id
        self.entities.append(entity)
        self.entity_lookup[entity.local_id] = entity

    def get_entity(self, local_id):
        return self.entity_lookup[local_id]

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

        self.rate = rospy.Rate(30)
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

        self.start()

        return EmptyResponse()

    def disable_callback(self, req):
        with self.lock:
            self.enabled = False

        return EmptyResponse()

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if not self.enabled:
                    break

                for source in self.sources:
                    source.publish_transforms()

            self.rate.sleep()

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

                parent_entity = attributes['parent']
                parent_module = parent_entity['module']
                parent_class = parent_entity['class']
                parent_tf_frame = parent_entity['tf_frame']
                local_id_attr = parent_entity['local_id_attr']

                source = HierarchicalEntityListSource(topic_name, module_name, class_name, attribute_name, parent_module, parent_class, parent_tf_frame, local_id_attr)

                for child in attributes['children']:
                    tf_frame = child['tf_frame']
                    pose_attr = child['pose_attr']
                    source.add_child_definition(tf_frame, pose_attr)

                sources.append(source)

        return sources


if __name__ == '__main__':
    rospy.init_node('perception_synthesiser')
    ps = PerceptionSynthesizer()
    rospy.spin()



