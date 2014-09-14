import rospy
import abc
from hri_msgs.msg import ExpressionAction
from hri_framework import MultiGoalActionServer


class ExpressionActionServer(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.action_server = MultiGoalActionServer('expression', ExpressionAction, auto_start=False)
        self.action_server.register_goal_callback(self.__goal_callback)
        self.action_server.register_preempt_callback(self.__preempt_callback)

    def start(self):
        self.action_server.start()

    def __goal_callback(self, goal_handle):
        goal = goal_handle.get_goal()
        self.start_expression(goal)
        rospy.loginfo("Expression received id: %s, name: %s", goal_handle.get_goal_id().id, goal.type)

    def __preempt_callback(self, goal_handle):
        goal = goal_handle.get_goal()
        self.stop_expression(goal)
        rospy.loginfo("Expression preempted id: %s, name: %s", goal_handle.get_goal_id().id, goal.type)

    @abc.abstractmethod
    def start_expression(self, goal):
        """ Start your expression """
        return

    @abc.abstractmethod
    def stop_expression(self, goal):
        """ Stop the expression corresponding to this goal """
        return

