#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
import rospy
import abc
from hri_msgs.msg import GazeAction, GazeActionFeedback
from threading import Thread
from actionlib.simple_action_server import SimpleActionServer


class GazeActionServer():
    __metaclass__ = abc.ABCMeta

    NODE_NAME = "gaze_action_server"

    def __init__(self):
        self.action_server = SimpleActionServer(GazeActionServer.NODE_NAME, GazeAction, auto_start=False)
        self.action_server.register_goal_callback(self.__goal_callback)
        self.gaze_thread = Thread(target=self.run)

    def start_server(self):
        rospy.init_node(GazeActionServer.NODE_NAME, anonymous=True)
        self.gaze_thread.start()
        self.action_server.start()

    def __goal_callback(self):
        self.gaze_goal = self.action_server.accept_new_goal()
        rospy.loginfo("Gaze goal received: " + str(self.gaze_goal))

    @abc.abstractmethod
    def run(self):
        """
            Run your gaze algorithm to make the robot gaze at a particular entity.
        """

    def send_feedback(self, distance_to_target):
        """ Call this method to send feedback about the distance to the target """
        feedback = GazeActionFeedback()
        feedback.distance_to_target = distance_to_target
        self.action_server.publish_feedback(self.feedback)
        rospy.loginfo("Gaze feedback. distance_to_target: $s", distance_to_target)

