#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
import rospy
from hri_msgs.srv import TfFrame, IfQueryableExecute, IfQueryableExecuteResponse


class EntityProxy():
    def __init__(self, entity_id):
        self.entity_id = entity_id
        self.tf_frame_service = rospy.ServiceProxy('tf_id_service', TfFrame)
        self.if_queryable_execute_service = rospy.ServiceProxy('if_queryable_execute', IfQueryableExecute)

    def tf_frame(self):
        tf_frame = "base_link"

        try:
            tf_frame = self.tf_frame_service(self.entity_id).tf_frame
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        return tf_frame

    def if_queryable_execute(self):
        try:
            response = self.if_queryable_execute_service(self.entity_id)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return None

        if response.is_queryable:
            entity_proxies = []

            for entity_id in response.entities:
                entity_proxies.append(EntityProxy(entity_id))

            return entity_proxies
        else:
            return None


