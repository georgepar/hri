

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
        mock.assert_called_with([5])