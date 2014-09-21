#!/usr/bin/env python
import rospy
import abc
from hri_msgs.msg import GestureAction, GestureActionFeedback
from threading import Timer
from hri_framework import MultiGoalActionServer





class IGestureActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self, gesture_enum):
        self.node_name = "gesture"
        self.gesture_enum = gesture_enum
        self.gesture_handle_lookup = {}
        self.action_server = MultiGoalActionServer(self.node_name, GestureAction, auto_start=False)
        self.action_server.register_goal_callback(self.__goal_callback)
        self.action_server.register_preempt_callback(self.__preempt_callback)

    def start(self):
        self.action_server.start()
        rospy.loginfo('GestureActionServer started')

    def is_valid_gesture(self, gesture_name):
        found = False
        for name, member in self.gesture_enum.__members__.items():
            if name == gesture_name:
                return True

        if not found:
            rospy.logerr('{0} is not a valid gesture. Valid gestures are: {1}'.format(gesture_name, self.gesture_enum.__members__.items()))
            return False

    def __goal_callback(self, goal_handle):
        new_goal = goal_handle.get_goal()
        self.start_gesture(goal_handle)
        rospy.loginfo("Gesture received id: %s, name: %s", goal_handle.get_goal_id().id, new_goal.gesture)

    def __preempt_callback(self, goal_handle):
        self.cancel_gesture(goal_handle)
        self.remove_gesture_handle(goal_handle)
        rospy.loginfo("Gesture preempted id: %s, name: %s", goal_handle.get_goal_id().id, goal_handle.get_goal().gesture)

    @abc.abstractmethod
    def start_gesture(self, goal_handle):
        """ Start your gesture. """
        return

    @abc.abstractmethod
    def cancel_gesture(self, goal_handle):
        """ Cancel gesture if it is currently running. """
        return

    def send_feedback(self, goal_handle, distance_to_target):
        """ Call this method to send feedback about the distance to the target """
        feedback = GestureActionFeedback()
        feedback.distance_to_target = distance_to_target
        self.action_server.publish_feedback(goal_handle, self.feedback)
        rospy.loginfo("Gesture feedback id: %s, name: %s, distance_to_target: $s", goal_handle.get_goal_id().id, goal_handle.get_goal().type, distance_to_target)

    def set_succeeded(self, goal_handle):
        """ Call this method when the gesture has finished """
        self.action_server.set_succeeded(goal_handle)
        self.remove_gesture_handle(goal_handle)
        rospy.loginfo("Gesture finished id: %s, name: %s",  goal_handle.get_goal_id().id, goal_handle.get_goal().gesture)

    def set_succeeded_on_timeout(self, goal_handle, timeout):
        gesture_handle = self.get_gesture_handle(goal_handle)
        gesture_handle.start_timer(timeout, self.set_succeeded, [goal_handle])

    def set_aborted(self, goal_handle):
        self.action_server.set_aborted()
        self.remove_gesture_handle(goal_handle)

    def add_gesture_handle(self, gesture_handle):
        self.gesture_handle_lookup[gesture_handle.goal_id] = gesture_handle

    def remove_gesture_handle(self, goal_handle):
        self.gesture_handle_lookup.pop(goal_handle.get_goal_id().id)

    def get_gesture_handle(self, goal_handle):
        return self.gesture_handle_lookup[goal_handle.get_goal_id().id]