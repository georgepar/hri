#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
import rospy
from actionlib import ActionServer, SimpleActionServer, GoalID
from hri_msgs.msg import GoalList
import threading
import abc


class MultiGoalActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self, name, ActionSpec, auto_start=True):
        self.goal_handle_lock = threading.RLock()

        self.preempt_request = False
        self.new_goal_preempt_request = False

        self.goal_callback = None
        self.preempt_callback = None
        self.goals = []

        self.active_goals_pub = rospy.Publisher(name + "/active_goals", GoalList)
        self.action_server = ActionServer(name, ActionSpec, self.internal_goal_callback, self.internal_preempt_callback, auto_start);

    def register_goal_callback(self, goal_callback):
        self.goal_callback = goal_callback

    def register_preempt_callback(self, preempt_callback):
        self.preempt_callback = preempt_callback

    def publish_active_goals(self):
        with self.goal_handle_lock:
            active_goals = GoalList()

            for goal_handle in self.goals:
                active_goals.goal_list.append(goal_handle.get_goal_id())
            self.active_goals_pub.publish(active_goals)

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
            self.goals.append(goal_handle)
            goal_handle.set_accepted("This goal has been accepted by the simple action server")
            self.publish_active_goals()

    def set_succeeded(self, goal_handle, result=None, text=""):
        with self.goal_handle_lock:
            self.goals.remove(goal_handle)
            if not result:
                result = self.get_default_result()
            goal_handle.set_succeeded(result, text)
            self.publish_active_goals()

    def set_aborted(self, goal_handle, result=None, text=""):
        with self.goal_handle_lock:
            self.goals.remove(goal_handle)
            if not result:
                result = self.get_default_result()
            goal_handle.set_aborted(result, text)
            self.publish_active_goals()

    def publish_feedback(self, goal_handle, feedback):
        with self.goal_handle_lock:
            goal_handle.publish_feedback(feedback)


