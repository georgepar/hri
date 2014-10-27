from unittest import TestCase
import rospy
import roslib
roslib.load_manifest("hri_api")
rospy.init_node("test2", anonymous=True)
from hri_msgs.msg import EntityMsg
from hri_api.entities import Saliency


class TestSaliency(TestCase):

    def test_create_saliency(self):
        msg = EntityMsg()
        msg.type = 'saliency'
        msg.number = 1
        saliency = Saliency.create_saliency(msg)
        self.assertEqual(saliency.global_frame_id(), 'saliency_1')
        self.assertEqual(saliency.default_body_part() , 'saliency_1')