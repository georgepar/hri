#! /usr/bin/env python

import roslib; roslib.load_manifest('hri_api')
import rospy
from std_msgs.msg import UInt16MultiArray
#from hri_api.srv import ExecuteQuery, GazeID, GestureID, IsQueryable, TFID, ExecuteQueryResponse, GazeIDResponse, GestureIDResponse, TFIDResponse, IsQueryableResponse
from hri_msgs.msg import EntityMsg, EntitiesMsg
import threading
from hri_api.entities import Entity
from hri_api.query import Query
from hri_api.query import is_callable
from hri_api.util import Singleton


class World():
    __metaclass__ = Singleton

    def __init__(self):
        rospy.init_node("world", anonymous= True)
        self.gaze_tf_id_service = rospy.Service('gaze_tf_id_service', GazeID, self.gaze_tf_id)
        self.gesture_tf_id_service = rospy.Service('gesture_tf_id_service', GestureID, self.gesture_tf_id)
        self.tf_id_service = rospy.Service('tf_id_service', TFID, self.tf_id)
        self.is_queryable_serivce = rospy.Service('is_queryable_serivce', IsQueryable, self.is_queryable)
        self.execute_query_service = rospy.Service('execute_query_service', ExecuteQuery, self.execute_query)

        self.entities = []
        self.entity_update_lock = threading.Lock()
        self.create_entity_callbacks = {}
        self.uuid_lookup = {}

    def subscribe_to_perception_topics(self):
        if rospy.has_param('~/perception_topics'):
            perception_topics = rospy.get_param('~/perception_topics')

            for topic in perception_topics:
                rospy.Subscriber(topic, EntitiesMsg, self.update_entities)
        else:
            raise Exception("Parameter ~perception_topics doesn't exist")

    def add_to_world(self, entity):
        uuid = id(entity)

        if isinstance(entity, Entity):
            if uuid not in self.uuid_entity_lookup:
                self.uuid_lookup[uuid] = entity
                self.entities.append(entity)
                rospy.logdebug("Added entity with uuid: %s", uuid)
        elif isinstance(entity, Query):
            if uuid not in self.uuid_query_lookup:
                self.uuid_lookup[uuid] = entity
                rospy.logdebug("Added query with uuid: %s", uuid)
        else:
            raise TypeError("add_to_world() parameter thing={0} is not a subclass of Entity or Query".format(entity))

    def add_create_entity_callback(self, entity_type, callback):
        if not isinstance(entity_type, str):
            raise TypeError("add_create_entity_callback() parameter entity_type={0} is not a str".format(entity_type))

        if not is_callable(callback):
            raise TypeError("add_create_entity_callback() parameter callback={0} is not callable".format(callback))

        self.create_entity_callbacks[entity_type] = callback

    def make_entity_from_entity_msg(self, entity_msg):
        if not isinstance(entity_msg, EntityMsg):
            raise TypeError("create_entity() parameter entity_msg={0} is not a subclass of EntityMsg".format(entity_msg))

        callback = self.create_entity_callbacks[entity_msg]
        return callback(entity_msg)

    def get_entity_from_uuid(self, uuid):
        if not isinstance(uuid, int):
            raise TypeError("get_entity_from_uuid() parameter uuid={0} is not a int".format(uuid))

        if uuid in self.uuid_lookup:
            return self.uuid_lookup[uuid]
        else:
            raise IndexError("get_entity_from_uuid() parameter uuid={0} is not in self.uuid_lookup".format(uuid))

    def gaze_tf_id(self, req):
        obj = self.get_entity_from_uuid(int(req.uri))
        tf_id = obj.say_to_gaze_tf_id()
        return GazeIDResponse(tf_id)

    def gesture_tf_id(self, req):
        obj = self.get_entity_from_uuid(int(req.uri))
        tf_id = obj.say_to_gesture_tf_id()
        return GestureIDResponse(tf_id)

    def tf_id(self, req):
        obj = self.get_entity_from_uuid(int(req.uri))
        tf_id = obj.tf_id()
        return TFIDResponse(tf_id)

    def is_queryable(self, req):
        obj = self.get_entity_from_uuid(int(req.uri))
        is_queryable = obj.is_queryable()
        return IsQueryableResponse(is_queryable)

    def execute_query(self, req):
        obj = self.get_entity_from_uuid(int(req.uri))
        uris = obj.execute_query()
        return ExecuteQueryResponse(uris)

    ''' Locked '''
    def update_entities(self, msg):
        with self.entity_update_lock:
            if not isinstance(msg, EntitiesMsg):
                raise TypeError("update_entities() parameter msg={0} is not an EntitiesMsg".format(msg))

            # Create new entities
            for entity_msg in msg.entities:
                entity_exists = False

                for entity_py in self.entities:
                    if entity_msg.type == entity_py.type and entity_msg.id == entity_py.id:
                        if not entity_py.is_visible():
                            rospy.logdebug('Setting entity to visible: %s', entity_msg)
                            entity_py.visible = True

                        entity_exists = True
                        break

                if not entity_exists:
                    rospy.logdebug('Creating entity: %s', entity_msg)
                    new_entity = self.make_entity_from_entity_msg(entity_msg)
                    self.add_to_world(new_entity)

            # Set entities that have dissapeared out of view to invisible
            for entity_py in self.entities:
                for entity_msg in msg.entities:
                    if entity_msg.type == entity_py.type and entity_msg.id == entity_py.id:
                        entity_py.visible = False
                        break
