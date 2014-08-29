#!/usr/bin/env python
import rospy
from hri_msgs.srv import TfFrame, IfQueryableExecute, IfQueryableExecuteResponse
from hri_framework.singleton import Singleton


class TfFrameService():
    __metaclass__ = Singleton

    def __init__(self):
        self.service = TfFrameService.connect()

    def call_service(self, entity_id):
        for i in range(1, 3):
            try:
                response = self.service(entity_id)
                break
            except rospy.ServiceException as exc:
                print("tf_frame_service: service did not process request: " + str(exc))
                self.service.close()
                self.service = TfFrameService.connect()

        return response.tf_frame

    @staticmethod
    def connect():
        rospy.loginfo('connecting to tf_frame_service')
        return rospy.ServiceProxy('tf_frame_service', TfFrame, persistent=True)


class IfQueryableExecuteService():
    __metaclass__ = Singleton

    def __init__(self):
        self.service = self.connect()

    def call_service(self, entity_id):
        for i in range(1, 3):
            try:
                response = self.service(entity_id)
                break
            except rospy.ServiceException as exc:
                print("if_queryable_execute: service did not process request: " + str(exc))
                self.service.close()
                self.service = IfQueryableExecuteService.connect()

        if response.is_queryable:
            entity_proxies = []
            for entity_id in response.entities:
                entity_proxies.append(EntityProxy(entity_id))
            return entity_proxies
        else:
            return None

    @staticmethod
    def connect():
        rospy.loginfo('connecting to if_queryable_execute')
        return rospy.ServiceProxy('if_queryable_execute', IfQueryableExecute, persistent=True)


class EntityProxy():
    def __init__(self, entity_id):
        self.entity_id = entity_id

    def tf_frame(self):
        service = TfFrameService()
        return service.call_service(self.entity_id)

    def if_queryable_execute(self):
        service = IfQueryableExecuteService()
        return service.call_service()


