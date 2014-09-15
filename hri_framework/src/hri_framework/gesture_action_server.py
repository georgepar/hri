#!/usr/bin/env python
import rospy
import abc
from hri_msgs.msg import GestureAction, GestureActionFeedback
from hri_framework.multi_goal_action_server import MultiGoalActionServer


class GestureActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.node_name = "gesture"
        self.action_server = MultiGoalActionServer(self.node_name, GestureAction, auto_start=False)
        self.action_server.register_goal_callback(self.__goal_callback)
        self.action_server.register_preempt_callback(self.__preempt_callback)

    def start_server(self):
        rospy.init_node(self.node_name, anonymous=True)
        self.action_server.start()

    def __goal_callback(self, goal_handle):
        new_goal = goal_handle.get_goal()
        self.start_gesture(new_goal)
        rospy.loginfo("Gesture received id: %s, name: %s", goal_handle.get_goal_id().id, new_goal.type)

    def __preempt_callback(self, goal_handle):
        self.cancel_gesture(goal_handle.get_goal_id().id)
        rospy.loginfo("Gesture preempted id: %s, name: %s", goal_handle.get_goal_id().id, goal_handle.get_goal().type)

    @abc.abstractmethod
    def start_gesture(self, gesture_action_msg):
        """ Start your gesture. """
        return

    @abc.abstractmethod
    def cancel_gesture(self, gesture_id):
        """ Cancel gesture if it is currently running. """
        return

    def send_feedback(self, goal_handle, distance_to_target):
        """ Call this method to send feedback about the distance to the target """
        feedback = GestureActionFeedback()
        feedback.distance_to_target = distance_to_target
        self.action_server.publish_feedback(goal_handle, self.feedback)
        rospy.loginfo("Gesture feedback id: %s, name: %s, distance_to_target: $s", goal_handle.get_goal_id().id, goal_handle.get_goal().type, distance_to_target)

    def gesture_finished(self, goal_handle):
        """ Call this method when the gesture has finished """
        self.action_server.set_succeeded(goal_handle)
        rospy.loginfo("Gesture finished id: %s, name: %s",  goal_handle.get_goal_id().id, goal_handle.get_goal().type)



