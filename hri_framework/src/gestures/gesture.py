#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
import rospy
import threading
import abc
#from action_servers import TfFrame, GestureActionServer
#from hri_framework.msg import GestureActionGoal

class Gesture():
    __metaclass__ = abc.ABCMeta

    #def __init__(self, GestureActionServer server, GestureActionGoal goal, double duration, TfFrame target):


    def __init__(self, server, goal_handle, double duration, target):
        self.server = server
        #self.lock = threading.RLock()
        self.goal_handle = goal_handle
        #self.body_lock_service = body_lock_service
        self.duration = duration
        self.target = target
        self.gesture_finished_callback = None
        self.gesture_feedback_callback = None
        self.gesture_thread = threading.Thread(target=self.run_gesture)
        self.gesture_thread.daemon = True

    #def make_gesture(cls, GestureActionServer server, GestureActionGoal goal, double duration, TfFrame target):

    @classmethod
    @abc.abstractmethod
    def make_gesture(cls, server, goal_handle, double duration, target):
        """
        Create a gesture object and return it.
        """
        return

    def set_finished(self):
        """
            Call this method when your gesture has finished.
        """

        if self.gesture_finished_callback is None:
            rospy.logdebug(
                "You haven't implement the gesture action server properly: please specify a gesture_finished_callback");
        else:
            self.gesture_finished_callback(self.goal_handle)

    def send_feedback(self, double distance_to_target):
        if self.gesture_finished_callback is None:
            rospy.logdebug(
                "You haven't implement the gesture action server properly: please specify a gesture_feedback_callback");
        else:
            self.gesture_finished_callback(self.goal_handle)

    @abc.abstractmethod
    def run_gesture(self):
        """
            Implement your code to run your gesture here.
        """

    def start(self):
        self.gesture_thread.start()

    @abc.abstractmethod
    def stop(self):
        """
            Stop your gesture here. Non-blocking.
        """

    def register_gesture_finished_callback(self, callback):
        self.gesture_finished_callback = callback

    def register_gesture_feedback_callback(self, callback):
        self.gesture_feedback_callback = callback

        #def request_body_lock(self):
        #    #response = self.body_lock_srv(self.body_parts_used())
        #    #return response.result
        #    return True

        #@abc.abstractmethod
        #def body_parts_used(self):
        #    """
        #     Return a list of the body parts used by this gesture.
        #    """
        #    return