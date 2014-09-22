#!/usr/bin/env python
import rospy
import abc
from hri_msgs.msg import TargetAction, TargetFeedback
from threading import Thread
from actionlib.simple_action_server import SimpleActionServer


class ITargetActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self, action_server_name):
        self.action_server_name = action_server_name
        self.action_server = SimpleActionServer(self.action_server_name, TargetAction, auto_start=False)
        self.action_server.register_goal_callback(self.__goal_callback)
        self.origin_frame = rospy.get_param('~origin_frame', 'base_link')
        self.success_distance = rospy.get_param('~success_distance', 0.2)
        self.end_effector_frame = rospy.get_param('~end_effector_frame', 'gaze')
        self.rate = rospy.Rate(rospy.get_param('~hz', 10))

    def start(self):
        self.action_server.start()

    def __goal_callback(self):
        target_goal = self.action_server.accept_new_goal()
        rospy.loginfo("Target goal received: " + str(target_goal))
        self.execute(target_goal)

    @abc.abstractmethod
    def execute(self, target_goal):
        """
            Run your gaze algorithm to make an end effector on the robot target a particular entity.
        """

    def send_feedback(self, distance_to_target):
        """ Call this method to send feedback about the distance to the target """

        feedback = TargetFeedback()
        feedback.distance_to_target = distance_to_target
        self.action_server.publish_feedback(feedback)
        rospy.loginfo("Target feedback. distance_to_target: {0}".format(distance_to_target))

