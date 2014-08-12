#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
import rospy
from hri_msgs.msg import TimerAction, TimerResult
from multi_goal_action_server import MultiGoalActionServer
import threading


class TimerActionServer(object):
    def __init__(self, node_name):
        self.action_server = MultiGoalActionServer(node_name, TimerAction, auto_start=False)
        self.action_server.register_goal_callback(self.goal_callback)
        self.action_server.register_preempt_callback(self.preempt_callback)
        self.timers = {}
        self.timers_lock = threading.RLock()

    def start_server(self):
        self.action_server.start()

    def goal_callback(self, goal_handle):
        print "TIMER STARTED"
        new_goal = goal_handle.get_goal()

        with self.timers_lock:
            goal_id = goal_handle.get_goal_id().id
            timer = threading.Timer(new_goal.duration, self.timer_stopped, [goal_handle])
            self.timers[goal_id] = timer
            timer.start()

    def preempt_callback(self, goal_handle):
        with self.timers_lock:
            goal_id = goal_handle.get_goal_id().id
            self.timers[goal_id].cancel()

        result = TimerResult()
        result.success = False
        goal_handle.set_canceled(self, result=result, text="Timer cancelled")

    def timer_stopped(self, goal_handle):
        print "TIMER STOPPED"
        result = TimerResult()
        result.success = True

        with self.timers_lock:
            goal_id = goal_handle.get_goal_id().id
            self.timers.pop(goal_id)

        self.action_server.set_succeeded(goal_handle, result=result, text="I succeeded!!!")

# if __name__ == '__main__':
#     server = TestMultiGoalActionServer()
#     server.start_server()
#     rospy.spin()