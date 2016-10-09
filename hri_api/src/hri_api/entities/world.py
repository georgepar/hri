#! /usr/bin/env python

import roslib; roslib.load_manifest('hri_api')
import rospy
from std_msgs.msg import UInt16MultiArray
#from hri_api.srv import ExecuteQuery, GazeID, GestureID, IsQueryable, TFID, ExecuteQueryResponse, GazeIDResponse, GestureIDResponse, TFIDResponse, IsQueryableResponse
from hri_msgs.msg import EntityMsg, EntityListMsg
import threading
from hri_api.entities import Entity
from hri_api.query import Query
from hri_api.query import is_callable
from hri_api.util import Singleton, InitNode
from hri_msgs.srv import TfFrame, TfFrameResponse, IfQueryableExecute, IfQueryableExecuteResponse, AddEntity, AddEntityResponse, SetVisibility, SetVisibilityResponse
from std_srvs.srv import Empty
import importlib


class World():
    __metaclass__ = Singleton

    def __init__(self):
        InitNode()
        self.tf_frame_service = rospy.Service('tf_frame_service', TfFrame, self.tf_frame_service_callback)
        self.if_queryable_execute_service = rospy.Service('if_queryable_execute', IfQueryableExecute, self.if_queryable_execute_callback)
        self.add_entity_srv = rospy.Service('add_entity', AddEntity, self.add_entity_callback)
        self.set_visibility_srv = rospy.Service('set_visibility', SetVisibility, self.set_visibility_callback)
        self.enable_perception_srv = rospy.ServiceProxy('perception_synthesiser/enable', Empty)
        self.disable_perception_srv = rospy.ServiceProxy('perception_synthesiser/disable', Empty)

        self.enable_perception_srv.wait_for_service()
        self.disable_perception_srv.wait_for_service()

        self.entity_lock = threading.RLock()
        self.entities = []
        self.entity_id_lookup = {}
        self.entity_classes = {}

        rospy.on_shutdown(self.shutdown)
        self.enable_perception_srv()

    def __iter__(self):
        return iter(self.entities)

    def shutdown(self):
        self.disable_perception_srv()

    def add_entity_callback(self, req):
        with self.entity_lock:
            module = importlib.import_module(req.entity_module)
            entity_cls = getattr(module, req.entity_class)
            entity = entity_cls.make(req.local_id)

            self.add_to_world(entity)
            resp = AddEntityResponse()
            resp.global_id = entity.get_id()
            rospy.loginfo('added entity {0} to World'.format(entity))
            return AddEntityResponse(resp.global_id)

    def set_visibility_callback(self, req):
        with self.entity_lock:
            entity = self.entity_from_entity_id(req.global_id)
            entity.set_visible(req.is_visible)
        return SetVisibilityResponse()

    def add_entity_class(self, cls, entity_type):
        self.entity_classes[entity_type] = cls

    def add_to_world(self, entity):
        entity_id = entity.get_id()

        if isinstance(entity, Entity):
            if entity_id not in self.entity_id_lookup:
                self.entity_id_lookup[entity_id] = entity
                self.entities.append(entity)
                rospy.logdebug("Added entity with entity_id: %s", entity_id)
        elif isinstance(entity, Query):
            if entity_id not in self.entity_id_lookup:
                self.entity_id_lookup[entity_id] = entity
                rospy.logdebug("Added query with entity_id: %s", entity_id)
        else:
            rospy.logerr("add_to_world() parameter entity={0} is not a subclass of Entity or Query".format(entity))

    def entity_from_entity_id(self, entity_id):
        if not isinstance(entity_id, str):
            rospy.logerr("get_entity_from_entity_id() parameter entity_id={0} is not a int".format(entity_id))

        if entity_id in self.entity_id_lookup:
            return self.entity_id_lookup[entity_id]
        else:
            rospy.logerr("get_entity_from_entity_id() parameter entity_id={0} is not in self.entity_id_lookup".format(entity_id))

    def tf_frame_service_callback(self, req):
        entity = self.entity_from_entity_id(req.entity_id)
        tf_frame = entity.tf_frame_id()
        return TfFrameResponse(tf_frame)

    def if_queryable_execute_callback(self, req):
        entity = self.entity_from_entity_id(req.entity_id)
        response = IfQueryableExecuteResponse()

        if isinstance(entity, Query):
            response.is_queryable = True
            entities = entity.execute()
            response.entities = World.to_entity_list_msg(entities)
        else:
            response.is_queryable = False

        return response

    @staticmethod
    def to_entity_list_msg(entities):
        entity_list_msg = EntityListMsg()

        for entity in entities:
            entity_msg = EntityMsg()
            entity_msg.entity_id = entity.get_id()
            entity_list_msg.entities.append(entity_msg)

        return entity_list_msg
