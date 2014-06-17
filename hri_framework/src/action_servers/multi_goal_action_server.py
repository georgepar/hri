#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
import rospy
from actionlib import ActionServer, SimpleActionServer
import threading
import abc


class MultiGoalActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self, name, ActionSpec, auto_start = True):
        self.goal_handle_lock = threading.RLock()

        self.preempt_request = False
        self.new_goal_preempt_request = False

        self.goal_callback = None
        self.preempt_callback = None
        self.goals = []

        self.action_server = ActionServer(name, ActionSpec, self.internal_goal_callback, self.internal_preempt_callback, auto_start);

    def register_goal_callback(self, goal_callback):
        self.goal_callback = goal_callback

    def register_preempt_callback(self, preempt_callback):
        self.preempt_callback = preempt_callback

    def internal_goal_callback(self, goal_handle):
        rospy.loginfo("Goal received: %s", str(id(goal_handle)))

        with self.goal_handle_lock:
            if self.goal_callback:
                self.set_accepted(goal_handle)
                self.goal_callback(goal_handle)
            else:
                raise Exception('goal_callback not registered')

    def internal_preempt_callback(self, goal_handle):
        rospy.loginfo("Preempt received: %s", str(id(goal_handle)))

        with self.goal_handle_lock:
            if self.preempt_callback:
                self.preempt_callback(goal_handle)
                self.set_aborted(goal_handle)
            else:
                 raise Exception('preempt_callback not registered')

    def set_accepted(self, goal_handle):
        with self.goal_handle_lock:
            rospy.loginfo("Accepting a new goal: %s", goal_handle)
            goal_handle.set_accepted("This goal has been accepted by the simple action server")

    def set_succeeded(self, goal_handle, result=None, text=""):
        with self.goal_handle_lock:
            if not result:
                result = self.get_default_result()
            goal_handle.set_succeeded(result, text)

    def set_aborted(self, goal_handle, result=None, text=""):
        with self.goal_handle_lock:
            if not result:
                result = self.get_default_result()
            goal_handle.set_aborted(result, text)

    def publish_feedback(self, goal_handle, feedback):
        with self.goal_handle_lock:
            goal_handle.publish_feedback(feedback)


