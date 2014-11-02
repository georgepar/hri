import rospy
from hri_msgs.msg import EntityMsg, EntityListMsg
import threading
from hri_api.entities import Entity
from hri_api.query import Query
from hri_api.util import Singleton, InitNode
from hri_msgs.srv import TfFrame, TfFrameResponse, IfQueryableExecute, IfQueryableExecuteResponse, AddEntity, AddEntityResponse, SetVisibility, SetVisibilityResponse
from std_srvs.srv import Empty
import importlib
from hri_api.util import ParamAssertions


class World():
    """The World class provides access to the entities perceived by the robots perception system. The World class
    is iterable, meaning that it can be iterated over just as a list or array can.

    Vendor specific perception algorithms communicate with the World instance via the APIs perception interface,
    composed of a the perception synthesiser node and the ROS tf library. The perception synthesiser tells the World
    when to create objects to represent perceived entities and the ROS tf library manages object coordinates.

    .. note::
        The World class uses the singleton design pattern. Once the World class has been instantiated, a reference
        to this instance will be returned if the constructor is called again: http://en.wikipedia.org/wiki/Singleton_pattern

    """

    __metaclass__ = Singleton

    def __init__(self):

        InitNode()
        self.tf_frame_service = rospy.Service('tf_frame_service', TfFrame, self.__tf_frame_service_callback)
        self.if_queryable_execute_service = rospy.Service('if_queryable_execute', IfQueryableExecute, self.__if_queryable_execute_callback)
        self.add_entity_srv = rospy.Service('add_entity', AddEntity, self.__add_entity_callback)
        self.set_visibility_srv = rospy.Service('set_visibility', SetVisibility, self.__set_visibility_callback)
        self.enable_perception_srv = rospy.ServiceProxy('perception_synthesiser/enable', Empty)
        self.disable_perception_srv = rospy.ServiceProxy('perception_synthesiser/disable', Empty)

        self.enable_perception_srv.wait_for_service()
        self.disable_perception_srv.wait_for_service()

        self.entity_lock = threading.RLock()
        self.entities = []
        self.global_id_lookup = {}

        rospy.on_shutdown(self.__shutdown)
        self.enable_perception_srv()

    def __iter__(self):
        return iter(self.entities)

    @staticmethod
    def to_entity_list_msg(entities):
        entity_list_msg = EntityListMsg()

        for entity in entities:
            entity_msg = EntityMsg()
            entity_msg.entity_id = entity.global_id()
            entity_list_msg.entities.append(entity_msg)

        return entity_list_msg

    def __shutdown(self):
        self.disable_perception_srv()

    def __add_entity_callback(self, req):
        with self.entity_lock:
            module = importlib.import_module(req.entity_module)
            entity_cls = getattr(module, req.entity_class)
            entity = entity_cls.make(req.local_id)

            self.add_to_world(entity)
            resp = AddEntityResponse()
            resp.global_id = entity.global_id()
            rospy.loginfo('added entity {0} to World'.format(entity))
            return AddEntityResponse(resp.global_id)

    def __set_visibility_callback(self, req):
        with self.entity_lock:
            entity = self.entity_from_global_id(req.global_id)
            entity.set_visible(req.is_visible)
        return SetVisibilityResponse()

    def add_to_world(self, entity):
        ParamAssertions.assert_types(self.add_to_world, entity, Entity, Query)

        global_id = entity.global_id()

        if isinstance(entity, Entity):
            if global_id not in self.global_id_lookup:
                self.global_id_lookup[global_id] = entity
                self.entities.append(entity)
                rospy.logdebug("Added entity with global_id: %s", global_id)
        elif isinstance(entity, Query):
            if global_id not in self.global_id_lookup:
                self.global_id_lookup[global_id] = entity
                rospy.logdebug("Added query with global_id: %s", global_id)

    def entity_from_global_id(self, global_id):
        ParamAssertions.assert_types(self.entity_from_global_id, global_id, str)
        self.entity_from_global_id.
        if global_id in self.global_id_lookup:
            return self.global_id_lookup[global_id]
        else:
            raise IndexError("World.{0}() parameter global_id={1} is not in self.global_id_lookup".format(self.entity_from_global_id.__name__, global_id))

    def __tf_frame_service_callback(self, req):
        entity = self.entity_from_global_id(req.entity_id)
        tf_frame = entity.global_frame_id()
        return TfFrameResponse(tf_frame)

    def __if_queryable_execute_callback(self, req):
        entity = self.entity_from_global_id(req.entity_id)
        response = IfQueryableExecuteResponse()

        if isinstance(entity, Query):
            response.is_queryable = True
            entities = entity.execute()
            response.entities = World.to_entity_list_msg(entities)
        else:
            response.is_queryable = False

        return response
