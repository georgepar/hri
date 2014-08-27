
import roslib

roslib.load_manifest('hri_framework')
import rospy
from unittest import TestCase, TestLoader
from hri_msgs.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback
from hri_api.actions import MultiGoalActionClient, ActionHandle
from hri_api.tests import TimerActionServer
from actionlib.action_client import GoalStatus
import time
from mock import Mock


__author__ = 'Jamie Diprose'


global count
count = 0


class TestMultiGoalActionClient(TestCase):
    ACTION_SERVER_NAME = "test_action_server"
    TIMEOUT = rospy.Duration(secs=1.0)

    def setUp(self):
        global count
        rospy.init_node("test", anonymous=True)
        self.action_server = TimerActionServer(TestMultiGoalActionClient.ACTION_SERVER_NAME + "_" + str(count))
        self.action_server.start_server()
        self.client = MultiGoalActionClient(TestMultiGoalActionClient.ACTION_SERVER_NAME + "_" + str(count), TimerAction)
        self.client.wait_for_server(timeout=TestMultiGoalActionClient.TIMEOUT)
        count += 1

    def tearDown(self):
        self.client.stop()

    def test_wait_for_server(self):
        started1 = self.client.wait_for_server(timeout=TestMultiGoalActionClient.TIMEOUT)

        client2 = MultiGoalActionClient("i_dont_exist", TimerAction)
        started2 = client2.wait_for_server(timeout=TestMultiGoalActionClient.TIMEOUT)

        self.assertEqual(started1, True)
        self.assertEqual(started2, False)

    def test_send_goal(self):
        timer = TimerGoal()
        timer.duration = 0.5
        gh1 = self.client.send_goal(timer)
        success = self.client.wait_for_result(gh1, timeout=rospy.Duration(secs=0.55))
        self.assertEqual(success, True)

    def test_get_result(self):
        timer = TimerGoal()
        timer.duration = 0.5
        gh1 = self.client.send_goal(timer)
        self.client.wait_for_result(gh1, timeout=rospy.Duration(secs=0.55))
        result = self.client.get_result(gh1)

        self.assertEqual(isinstance(result, TimerResult), True)
        self.assertEqual(result.success, True)

    def test_send_two_goals_serial(self):
        # First goal
        timer1 = TimerGoal()
        timer1.duration = 0.5
        gh1 = self.client.send_goal(timer1)
        success1 = self.client.wait_for_result(gh1, timeout=rospy.Duration(secs=0.6))

        # Second goal
        timer2 = TimerGoal()
        timer2.duration = 0.5
        gh2 = self.client.send_goal(timer2)
        success2 = self.client.wait_for_result(gh2, timeout=rospy.Duration(secs=0.6))

        self.assertEqual(success1, True)
        self.assertEqual(success2, True)

    def test_send_two_goals_parallel(self):
        timer1 = TimerGoal()
        timer2 = TimerGoal()
        timer1.duration = 1.0
        timer2.duration = 1.0

        start = time.time()
        # Send both goals
        gh1 = self.client.send_goal(timer1)
        gh2 = self.client.send_goal(timer2)
        result1 = self.client.wait_for_result(gh1, timeout=rospy.Duration(secs=1.1))
        result2 = self.client.wait_for_result(gh2, timeout=rospy.Duration(secs=1.1))
        end = time.time()
        duration = end - start

        self.assertEqual(result1, True)
        self.assertEqual(result2, True)
        self.assertAlmostEqual(duration, 1.0, places=1)

    def test_get_goal_id(self):
        timer = TimerGoal()
        timer.duration = 0.1
        gh1 = self.client.send_goal(timer)
        goal_id = self.client.get_goal_id(gh1)
        self.assertIsNotNone(goal_id)

    def test_is_tracking_goal(self):
        timer = TimerGoal()
        timer.duration = 0.1
        gh1 = self.client.send_goal(timer)
        tracking = self.client.is_tracking_goal(gh1)
        self.assertEqual(tracking, True)

    def test_get_action_handle(self):
        timer = TimerGoal()
        timer.duration = 1.0
        gh1 = self.client.send_goal(timer)
        time.sleep(0.5)
        action_handle = self.client.get_action_handle(gh1)
        self.assertEqual(isinstance(action_handle, ActionHandle), True)

    def test_get_state(self):
        timer = TimerGoal()
        timer.duration = 1.0
        gh1 = self.client.send_goal(timer)
        pending = self.client.get_state(gh1)
        time.sleep(0.5)
        active = self.client.get_state(gh1)
        self.client.wait_for_result(gh1, timeout=rospy.Duration(secs=1.0))
        succeeded = self.client.get_state(gh1)

        # gh2 = self.client.send_goal(timer)
        # time.sleep(0.5)
        # self.client.cancel_goal(gh2)
        #preeempted = self.client.get_state(gh2)

        self.assertEqual(pending, GoalStatus.PENDING)
        self.assertEqual(active, GoalStatus.ACTIVE)
        self.assertEqual(succeeded, GoalStatus.SUCCEEDED)
        # self.assertEqual(preeempted, GoalStatus)

    def test_cancel_goal(self):
        timer1 = TimerGoal()
        timer1.duration = 5.0

        gh1 = self.client.send_goal(timer1)
        pending = self.client.get_state(gh1); time.sleep(0.5)
        active = self.client.get_state(gh1); self.client.cancel_goal(gh1); time.sleep(0.5)
        preempted = self.client.get_state(gh1)

        self.assertEqual(GoalStatus.PENDING, pending)
        self.assertEqual(GoalStatus.ACTIVE, active)
        self.assertEqual(GoalStatus.PREEMPTED, preempted)

    def test_cancel_all_goals(self):
        timer1 = TimerGoal()
        timer2 = TimerGoal()
        timer1.duration = 1.0
        timer2.duration = 1.0

        gh1 = self.client.send_goal(timer1)
        gh2 = self.client.send_goal(timer2)
        time.sleep(0.5)
        self.client.cancel_all_goals()
        time.sleep(0.5)

        self.assertEqual(GoalStatus.PREEMPTED, self.client.get_state(gh1))
        self.assertEqual(GoalStatus.PREEMPTED, self.client.get_state(gh2))

    def test_cancel_goals_at_and_before_time(self):
        before_time1 = TimerGoal()
        before_time2 = TimerGoal()
        before_time1.duration = 5.0
        before_time2.duration = 5.0
        gh1 = self.client.send_goal(before_time1)
        gh2 = self.client.send_goal(before_time2)
        cancel_time = rospy.Time().now()

        after_time_1 = TimerGoal()
        after_time_1.duration = 1.0
        gh3 = self.client.send_goal(after_time_1)
        time.sleep(0.5)
        self.client.cancel_goals_at_and_before_time(cancel_time)

        success = self.client.wait_for_result(gh3, timeout=rospy.Duration(secs=1.1))

        self.assertEqual(GoalStatus.PREEMPTED, self.client.get_state(gh1))
        self.assertEqual(GoalStatus.PREEMPTED, self.client.get_state(gh2))
        self.assertEqual(GoalStatus.SUCCEEDED, self.client.get_state(gh3))
        self.assertEqual(success, True)

    def test_done_callback(self):
        timer = TimerGoal()
        timer.duration = 0.5
        mock = Mock()
        gh1 = self.client.send_goal(timer, done_cb=mock)
        time.sleep(1.0)
        mock.assert_called_once_with(gh1)

    def test_active_callback(self):
        timer = TimerGoal()
        timer.duration = 0.5
        mock = Mock()
        gh1 = self.client.send_goal(timer, active_cb=mock)
        self.client.wait_for_result(gh1, timeout=rospy.Duration(secs=1.0))
        mock.assert_called_once_with(gh1)

    def test_feedback_callback(self):
        timer = TimerGoal()
        timer.duration = 1.0
        mock = Mock()
        gh1 = self.client.send_goal(timer, feedback_cb=mock)
        self.client.wait_for_result(gh1, timeout=rospy.Duration(secs=1.1))
        mock.assert_called_once_with(gh1, TimerFeedback(current_time=10))

        #calls = [TimerFeedback(current_time=0.2), TimerFeedback(current_time=0.4), TimerFeedback(current_time=0.6), TimerFeedback(current_time=0.8)]
        #mock.assert_has_calls(mock, calls)
    #
    # def test_send_goal_and_wait(self):
    #     self.fail()
    #
    # def test_wait_for_result(self):
    #     self.fail()
    #
    # def test_get_goal_status_text(self):
    #     self.fail()






