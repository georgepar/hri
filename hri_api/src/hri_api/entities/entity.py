#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from .abstract_entity import AbstractEntity
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


class Entity(AbstractEntity):

    def __init__(self, entity_type, tf_frame_prefix, parent):
        InitNode()
        self.tl = tf.TransformListener()
        self.entity_type = entity_type
        self.tf_frame_prefix = tf_frame_prefix
        self.parent = parent
        self.visible = True

    # def __str__(self):
    #     return self.tf_frame_id()

    def __repr__(self):
        return self.get_id()

    def __eq__(self, other):
        if self.tf_frame_id() == other.tf_frame_id():
            return True
        return False

    def is_visible(self):
        return self.visible

    def set_visible(self, visible):
        self.visible = visible

    def get_id(self):
        return str(id(self))

    def default_tf_frame_id(self):
        msg = "Entity.default_tf_frame_id: Please implement this method"
        rospy.logerr(msg)
        raise NotImplementedError(msg)

    def tf_frame_id(self):
        if self.parent is None:
            return self.tf_frame_prefix
        else:
            return self.parent.tf_frame_id() + '_' + self.tf_frame_prefix

    def translation_to(self, target):
        if not isinstance(target, AbstractEntity):
            msg = "translation_to() parameter target={0} is not a subclass of AbstractEntity".format(target)
            rospy.logerr(msg)
            raise TypeError(msg)

        try:
            (trans, rot) = self.tl.lookupTransform(self.default_tf_frame_id(), target.default_tf_frame_id(), rospy.Time())
            point = Point(trans[0], trans[1], trans[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            point = Point()
            rospy.loginfo("Couldn't transform from '" + self.tf_frame_id() + "' to '" + target.tf_frame_id() + "'")

        return point

    def is_infront_of(self, other_entity):
        if not isinstance(other_entity, AbstractEntity):
            msg = "is_infront_of() parameter other_entity={0} is not a subclass of AbstractEntity".format(other_entity)
            rospy.logerr(msg)
            raise TypeError(msg)

        origin = Point()
        other = self.translation_to(other_entity)
        return GeomMath.is_infront_of(other, origin)

    def is_behind(self, other_entity):
        if not isinstance(other_entity, AbstractEntity):
            msg = "is_behind() parameter other_entity={0} is not a subclass of AbstractEntity".format(other_entity)
            rospy.logerr(msg)
            raise TypeError(msg)

        origin = Point()
        other = self.translation_to(other_entity)
        return GeomMath.is_behind(other, origin)

    def is_left_of(self, other_entity):
        if not isinstance(other_entity, AbstractEntity):
            msg = "is_behind() parameter other_entity={0} is not a subclass of AbstractEntity".format(other_entity)
            rospy.logerr(msg)
            raise TypeError(msg)

        origin = Point()
        other = self.translation_to(other_entity)
        return GeomMath.is_left_of(other, origin)

    def is_right_of(self, other_entity):
        if not isinstance(other_entity, AbstractEntity):
            msg = "is_right_of() parameter other_entity={0} is not a subclass of AbstractEntity".format(other_entity)
            rospy.logerr(msg)
            raise TypeError(msg)

        origin = Point()
        other = self.translation_to(other_entity)
        return GeomMath.is_right_of(other, origin)

    def distance_to(self, other_entity):
        if not isinstance(other_entity, AbstractEntity):
            msg = "distance_to() parameter other_entity={0} is not a subclass of AbstractEntity".format(other_entity)
            rospy.logerr(msg)
            raise TypeError(msg)

        origin = Point()
        other = self.translation_to(other_entity)
        return GeomMath.distance_between(origin, other)

    def velocity(self, other_entity):
        return 0.1

    @staticmethod
    def wait_for_services(*services):
        for i, service in enumerate(services):
            if not isinstance(service, ServiceProxy):
                msg = "wait_for_services() parameter action_servers[{0}]={1} is not a ServiceProxy".format(i, service)
                rospy.logerr(msg)
                raise TypeError(msg)

            rospy.loginfo("Waiting for service: %s", service.resolved_name)
            service.wait_for_service()
            rospy.loginfo("Service found: %s", service.resolved_name)

        rospy.loginfo("All services found")

    @staticmethod
    def wait_for_action_servers(*action_servers):
        for i, action_server in enumerate(action_servers):
            if not isinstance(action_server, (actionlib.SimpleActionClient, MultiGoalActionClient)):
                msg = "wait_for_action_servers() parameter action_servers[{0}]={1} is not a SimpleActionClient or MultiGoalActionClient".format(i, action_server)
                rospy.logerr(msg)
                raise TypeError(msg)

            name = action_server.action_client.ns
            rospy.loginfo("Waiting for action server: %s", name)
            action_server.wait_for_server()
            rospy.loginfo("Action server: %s found", name)

        rospy.loginfo("All action servers found")

    @classmethod
    def make(cls, entity_num):
        """
        make a class and return it
        """
        msg = 'Entity.make: Please implement my make method'
        rospy.logerr(msg)
        raise NotImplementedError(msg)