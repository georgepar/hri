#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
import rospy
from actionlib import ActionServer, SimpleActionServer
import threading
import abc
from hri_framework.msg import GestureAction
from hri_framework.action_servers import TfFrame
from hri_framework.action_servers import RobotParams
from hri_framework.msg import GestureActionGoal

#TODO: Thank grob

class GestureActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.gestures = {}

        # since the internal_goal/preempt_callbacks are invoked from the
        # ActionServer while holding the self.action_server.lock
        # self.lock must always be locked after the action server lock
        # to avoid an inconsistent lock acquisition order
        self.goal_handle_lock = threading.RLock()

    def start_server(self):
        print "start node"
        rospy.init_node("gesture_action_server", anonymous=True)
        print "node done"
        self.robot_params = RobotParams()
        print "params done"

        self.action_server = ActionServer(self.robot_params.robot_id() + '_gesture', GestureAction, self.internal_goal_callback, self.internal_preempt_callback, False)

    @abc.abstractmethod
    def gesture_class_from_gesture_type(self, gesture_type):
        """
            Return a gesture class
        """
        return

    def internal_goal_callback(self, goal_handle):
        print "HANDLE ID: " + str(id(goal_handle))

        with self.goal_handle_lock:
            new_goal = goal_handle.get_goal()

            try:
                rospy.loginfo("New goal arrived")
                self.set_accepted(goal_handle)

                gesture_class = self.gesture_class_from_gesture_type(new_goal.gesture_type)

                if gesture_class is None:
                    self.set_aborted(goal_handle)
                    self.remove_goal(goal_handle)
                    rospy.loginfo("NaoGestureActionServer does not have a definition for gesture %s", new_goal.gesture_type)
                else:
                    gesture = gesture_class.make_gesture(self, goal_handle, new_goal.duration, TfFrame(new_goal.target_obj_id))
                    gesture.register_gesture_feedback_callback(self.send_feedback_callback)
                    gesture.register_gesture_finished_callback(self.gesture_finished_callback)
                    #goal_handle.gesture = gesture
                    print "before"
                    #if gesture.request_body_lock():
                    self.add_gesture(goal_handle.get_goal_id().id, gesture)
                    gesture.start()
                    print "after"
                    #else:
                    #    self.set_aborted(goal_handle)
                    #    self.remove_goal(goal_handle)
                    #    rospy.logwarn("Gesture " + str(gesture.gesture_id) + " couldn't get BodyLock.")

                    rospy.loginfo("Goal accepted")
                    print "keep goin"

            except Exception, e:
                rospy.logerr("SimpleActionServer.internal_goal_callback - exception %s",str(e))

        #self.execute_condition.release()

    def internal_preempt_callback(self, goal_handle):
        print "HANDLE ID: " + str(id(goal_handle))
        with self.goal_handle_lock:
            gesture = self.gestures[goal_handle.get_goal_id().id]
            gesture.stop()
            self.set_aborted(goal_handle)
            self.remove_gesture(goal_handle.get_goal_id().id)
            rospy.loginfo("Goal ABORTED")

    def send_feedback_callback(self, goal_handle, distance_to_target):
        with self.goal_handle_lock:
           # with goal_handle.lock:
                #self.feedback.distance_to_target = distance_to_target

            #TODO: make feedback message
            self.publish_feedback(goal_handle, "")
            rospy.loginfo("Feedback sent.")

    def gesture_finished_callback(self, goal_handle):
        print "HANDLE ID: " + str(id(goal_handle))
        with self.goal_handle_lock:
            self.set_succeeded(goal_handle)
            self.remove_gesture(goal_handle.get_goal_id().id)
            print "succeeded"

    def set_accepted(self, goal_handle):
        with self.goal_handle_lock:
            rospy.loginfo("Accepting a new goal");
            goal_handle.set_accepted("This goal has been accepted by the simple action server") #set the status of the current goal to be active

    ## @brief Sets the status of the active goal to succeeded
    ## @param  result An optional result to send back to any clients of the goal
    def set_succeeded(self, goal_handle, result=None, text=""):
      with self.goal_handle_lock:
          if not result:
              result=self.get_default_result()
          goal_handle.set_succeeded(result, text)

    ## @brief Sets the status of the active goal to aborted
    ## @param  result An optional result to send back to any clients of the goal
    def set_aborted(self, goal_handle, result = None, text=""):
        with self.goal_handle_lock:
            if not result:
                result=self.get_default_result()
            goal_handle.set_aborted(result, text)

    ## @brief Publishes feedback for a given goal
    ## @param  feedback Shared pointer to the feedback to publish
    def publish_feedback(self, goal_handle, feedback):
        with self.goal_handle_lock:
            goal_handle.publish_feedback(feedback)

    def get_default_result(self):
        return self.action_server.ActionResultType()

    def add_gesture(self, goal_id, gesture):
        with self.goal_handle_lock:
            self.gestures[goal_id] = gesture

    def remove_gesture(self, goal_id):
        with self.goal_handle_lock:
            del self.gestures[goal_id]









