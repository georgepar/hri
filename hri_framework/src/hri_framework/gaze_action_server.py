#!/usr/bin/env python
import rospy
import abc
from hri_msgs.msg import GazeAction, GazeFeedback
from threading import Thread
from actionlib.simple_action_server import SimpleActionServer


class IGazeActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.action_server_name = 'gaze'
        self.action_server = SimpleActionServer(self.action_server_name, GazeAction, auto_start = False)
        self.action_server.register_goal_callback(self.__goal_callback)
        self.origin_frame = rospy.get_param('~origin_frame', 'base_link')
        self.gaze_frame = rospy.get_param('~gaze_frame', 'gaze')
        self.rate = rospy.Rate(rospy.get_param('~hz', 10))

    def start(self):
        self.action_server.start()

    def __goal_callback(self):
        gaze_goal = self.action_server.accept_new_goal()
        rospy.loginfo("Gaze goal received: " + str(gaze_goal))
        self.execute(gaze_goal)

    @abc.abstractmethod
    def execute(self, gaze_goal):
        """
            Run your gaze algorithm to make the robot gaze at a particular entity.
        """

    def send_feedback(self, distance_to_target):
        """ Call this method to send feedback about the distance to the target """

        feedback = GazeFeedback()
        feedback.distance_to_target = distance_to_target
        self.action_server.publish_feedback(feedback)
        rospy.loginfo("Gaze feedback. distance_to_target: {0}".format(distance_to_target))

