from unittest import TestCase
import roslib
roslib.load_manifest('hri_framework')
import rospy
import abc
from hri_msgs.msg import TestAction, TestGoal, TestActionGoal, TestActionFeedback
from hri_framework.multi_goal_action_server import MultiGoalActionServer
from actionlib import SimpleActionClient
from mock import Mock


class TestMultiGoalActionServer(TestCase):
    def test_multi_goal_action_server(self):
        rospy.init_node('test', anonymous=True)
        mock = Mock()
        node_name = "test_action_server"
        action_server = MultiGoalActionServer(node_name, TestAction, auto_start=False)
        action_server.register_goal_callback(mock)
        action_server.start()

        timer = TestGoal()
        timer.duration = 5.0
        client = SimpleActionClient(node_name, TestAction)
        client.send_goal(timer)


