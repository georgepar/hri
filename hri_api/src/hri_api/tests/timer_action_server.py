#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
from hri_msgs.msg import TimerAction, TimerResult, TimerFeedback
from hri_framework.multi_goal_action_srv import MultiGoalActionServer
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
            self.action_server.publish_feedback(goal_handle, TimerFeedback(current_time=10))

    def preempt_callback(self, goal_handle):
        with self.timers_lock:
            goal_id = goal_handle.get_goal_id().id
            self.timers[goal_id].cancel()
            self.action_server.set_preempted(goal_handle)

    def timer_stopped(self, goal_handle):
        print "TIMER STOPPED"
        result = TimerResult()
        result.success = True

        with self.timers_lock:
            goal_id = goal_handle.get_goal_id().id
            self.timers.pop(goal_id)

        self.action_server.set_succeeded(goal_handle, result=result, text="I succeeded!!!")
#
# class Timer():
#     def __init__(self, duration, intervals):
#         self.duration = duration
#         self.intervals = intervals
#         self.preempt_lock = threading.RLock()
#         self.preempted = False
#         self.thread = None
#
#     def get_hz(self):
#         return (1.0 / self.duration) * self.intervals
#
#
# class TimerActionServer(object):
#     def __init__(self, node_name):
#         self.action_server = MultiGoalActionServer(node_name, TimerAction, auto_start=False)
#         self.action_server.register_goal_callback(self.goal_callback)
# #        self.action_server.register_preempt_callback(self.preempt_callback)
#         self.timers = {}
#         self.timers_lock = threading.RLock()
#
#     def start_server(self):
#         self.action_server.start()
#
#     def goal_callback(self, goal_handle):
#         print "TIMER STARTED"
#         new_goal = goal_handle.get_goal()
#
#         with self.timers_lock:
#             goal_id = goal_handle.get_goal_id().id
#             timer = Timer(new_goal.duration, 5)
#             thread = threading.Thread(target=self.run_timer, args=[goal_handle, timer])
#             timer.thread = thread
#             self.timers[goal_id] = timer
#             thread.start()
#
#     def run_timer(self, goal_handle, timer):
#         goal_id = goal_handle.get_goal_id().id
#         hz = timer.get_hz()
#         rate = rospy.Rate(hz)
#         i = 0
#
#         while i < timer.intervals and not rospy.is_shutdown():
#             with self.timers_lock:
#                 if self.action_server.is_preempt_requested(goal_handle):
#                     self.action_server.set_preempted(goal_handle)
#                     self.timers.pop(goal_id)
#                     return
#
#                 feedback = TimerFeedback(current_time=(i * (1.0 / timer.intervals)))
#                 self.action_server.publish_feedback(goal_handle, feedback)
#                 i += 1
#
#             rate.sleep()
#
#         result = TimerResult()
#         result.success = True
#         self.action_server.set_succeeded(goal_handle, result=result, text="I succeeded!!!")
#
#         with self.timers_lock:
#             self.timers.pop(goal_id)

    # def preempt_callback(self, goal_handle):
    #     with self.timers_lock:
    #         goal_id = goal_handle.get_goal_id().id
    #         timer = self.timers[goal_id]
    #         timer.thread.exit()


