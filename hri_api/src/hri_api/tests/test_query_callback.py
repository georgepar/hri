

import unittest
import rospy
from mock import Mock
from hri_api.query import Query


class TestQueryCallback(unittest.TestCase):

    def test_query_callback(self):
        rospy.init_node('test_node', anonymous=True)

        mock = Mock()
        a = [1, 2, 3, 4]
        query = Query(a).select(lambda x: x > 4)
        query.subscribe(mock)
        a.append(5)
        rospy.sleep(0.1)
        mock.assert_called_with([5])
        query.stop()

        mock.reset_mock()
        b = []
        query = Query(b).order_by_ascending().take(2)
        query.subscribe(mock)

        b.append(1)
        rospy.sleep(0.1)
        mock.assert_called_with([1])

        b.append(2)
        rospy.sleep(0.1)
        mock.assert_called_with([1, 2])

        b.append(5)
        rospy.sleep(0.1)
        mock.assert_called_with([1, 2])

        b.append(-10)
        rospy.sleep(0.1)
        mock.assert_called_with([-10, 1])


