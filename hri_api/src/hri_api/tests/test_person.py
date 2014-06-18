from unittest import TestCase
import rospy
import roslib
roslib.load_manifest("hri_api")
rospy.init_node("test2", anonymous=False)

from hri_msgs.msg import EntityMsg
from hri_api.entities import Person


class TestPerson(TestCase):
    def test_create_person(self):
        msg = EntityMsg()
        msg.type = 'person'
        msg.number = 1
        person = Person.create_person(msg)
        self.assertEqual(person.tf_frame_id(), 'person_1')
        self.assertEqual(person.neck.tf_frame_id(), 'person_1_neck')
        self.assertEqual(person.head.tf_frame_id(), 'person_1_head')
        self.assertEqual(person.torso.tf_frame_id(), 'person_1_torso')
        self.assertEqual(person.left_hand.tf_frame_id(), 'person_1_left_hand')
        self.assertEqual(person.right_hand.tf_frame_id(), 'person_1_right_hand')
        self.assertEqual(person.default_tf_frame_id(), 'person_1_torso')
