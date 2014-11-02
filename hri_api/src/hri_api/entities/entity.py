#!/usr/bin/env python
import tf
import rospy
from geometry_msgs.msg import Point
from hri_api.math import GeomMath
from rospy import ServiceProxy
import actionlib
import abc
import math
from hri_api.util import InitNode
from hri_api.actions import MultiGoalActionClient
import uuid
from hri_api.util import ParamAssertions
import sys
import enum


class NamingScheme(enum):

    Flat = 1
    Hierarchical = 2


class Entity(object):

    def __init__(self, local_frame_id, parent, naming_scheme=NamingScheme.Hierarchical):
        InitNode()

        ParamAssertions.assert_types(self.__init__, local_frame_id, str)
        ParamAssertions.assert_types(self.__init__, parent, Entity)
        ParamAssertions.assert_types(self.__init__, naming_scheme, NamingScheme)

        self.tl = tf.TransformListener()
        self.local_frame_id = local_frame_id
        self.parent = parent
        self.visible = True
        self._global_id = str(uuid.uuid4())
        self.naming_scheme = naming_scheme
        self.children = []

        # Adds self to parent to create parent -> child relationship
        if parent is not None:
            parent.add_child(self)

    def add_child(self, child):
        """

        Adds a child to the entity.

        :param child: the child
        :type child: Entity
        :raises TypeError: child must be a subtype of Entity
        """

        ParamAssertions.assert_types(self.add_child, child, Entity)
        self.children.append(child)

    @property
    def global_id(self):
        """

        :return: Get the entities global id. The global id is used to uniquely identify the entity. Note that this  It uniquely identifies this entit
        :rtype: str
        """

        return self._global_id

    @classmethod
    def make(cls, local_id):
        """

        Instantiate a derived Entity class, e.g. Person(local_id) and return it.

        :param local_id: the id that uniquely identifies the entity amongst all entities of the same type
        :type local_id: int
        :return: the entity instance
        :rtype: Entity
        :raises NotImplementedError: please implement this method
        """

        raise NotImplementedError('please implement this method')

    def default_body_part(self):
        """

        Gets the default body part to use for math calculations. Only applicable to entities with children.

        :return: the default body part to use for math calculations.
        :rtype: Entity
        :raises NotImplementedError: please implement this method
        """

        if self.parent is None and len(self.children) > 0:
            raise NotImplementedError("Please implement this method")

    def global_frame_id(self, depth=0):
        """

        Gets the global frame id. Only applicable to entities with children.

        :return: the default body part to use for math calculations.
        :rtype: str
        :raises NotImplementedError: please implement this method
        """

        if self.naming_scheme is NamingScheme.Flat:
            return self.local_frame_id

        elif self.naming_scheme is NamingScheme.Hierarchical:
            if depth == 0 and self.parent is None and len(self.children) > 0:
                return self.default_body_part().global_frame_id()

            elif depth == 0 and self.parent is None:
                return self.local_frame_id

            else:
                return self.parent.global_frame_id(depth + 1) + '_' + self.local_frame_id

    def translation_to(self, target):
        ParamAssertions.assert_types(self.translation_to, target, Entity)

        try:
            (trans, rot) = self.tl.lookupTransform(self.default_body_part(), target.default_body_part(), rospy.Time())
            point = Point(trans[0], trans[1], trans[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            point = Point()
            rospy.loginfo("Couldn't transform from '" + self.global_frame_id() + "' to '" + target.global_frame_id() + "'")

        return point

    def infront_of(self, entity):
        """

        Check if the current instance is in front of entity

        :param entity:
        :type entity: Entity
        :return: whether this assertion is true or not
        :rtype: bool
        :raises TypeError: entity is not of type Entity
        """

        ParamAssertions.assert_types(self.infront_of, entity, Entity)

        origin = Point()
        other = self.translation_to(entity)

        if other.x >= origin.x:
            return True
        return False

    def behind(self, entity):
        """

        Check if the current instance is behind entity

        :param entity:
        :type entity: Entity
        :return: whether this assertion is true or not
        :rtype: bool
        :raises TypeError: entity is not of type Entity
        """

        ParamAssertions.assert_types(self.behind, entity, Entity)

        origin = Point()
        other = self.translation_to(entity)

        if other.x < origin.x:
            return True
        return False

    def left_of(self, entity):
        """

        Check if the current instance is left_of entity

        :param entity:
        :type entity: Entity
        :return: whether this assertion is true or not
        :rtype: bool
        :raises TypeError: entity is not of type Entity
        """

        ParamAssertions.assert_types(self.left_of, entity, Entity)

        origin = Point()
        other = self.translation_to(entity)

        if other.y >= origin.y:
            return True
        return False

    def right_of(self, entity):
        """

        Check if the current instance is right_of entity

        :param entity:
        :type entity: Entity
        :return: whether this assertion is true or not
        :rtype: bool
        :raises TypeError: entity is not of type Entity
        """

        ParamAssertions.assert_types(self.right_of, entity, Entity)

        origin = Point()
        other = self.translation_to(entity)

        if other.y < origin.y:
            return True
        return False

    def distance_to(self, entity):
        """

        Calculate the distance from the current instance to entity

        :param entity:
        :type entity: Entity
        :return: the distance from the instance to entity
        :rtype: float
        :raises TypeError: entity is not of type Entity
        """

        ParamAssertions.assert_types(self.distance_to, entity, Entity)

        origin = Point()
        other = self.translation_to(entity)

        x_diff = origin.x - other.x
        y_diff = origin.y - other.y
        z_diff = origin.z - other.z
        return math.sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff)

    def __repr__(self):
        return self.global_id

    def __eq__(self, other):
        if self.global_id == other.global_id:
            return True
        return False

    @staticmethod
    def wait_for_services(*services):
        for i, service in enumerate(services):
            if not isinstance(service, ServiceProxy):
                raise TypeError("wait_for_services() parameter action_servers[{0}]={1} is not a ServiceProxy".format(i, service))

            rospy.loginfo("Waiting for service: %s", service.resolved_name)
            service.wait_for_service()
            rospy.loginfo("Service found: %s", service.resolved_name)

        rospy.loginfo("All services found")

    @staticmethod
    def wait_for_action_servers(*action_servers):
        for i, action_server in enumerate(action_servers):
            if not isinstance(action_server, (actionlib.SimpleActionClient, MultiGoalActionClient)):
                raise TypeError("wait_for_action_servers() parameter action_servers[{0}]={1} is not a SimpleActionClient or MultiGoalActionClient".format(i, action_server))

            name = action_server.action_client.ns
            rospy.loginfo("Waiting for action server: %s", name)
            action_server.wait_for_server()
            rospy.loginfo("Action server: %s found", name)

        rospy.loginfo("All action servers found")

