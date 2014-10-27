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
        self.assertEqual(person.global_frame_id(), 'person_1')
        self.assertEqual(person.neck.global_frame_id(), 'person_1_neck')
        self.assertEqual(person.head.global_frame_id(), 'person_1_head')
        self.assertEqual(person.torso.global_frame_id(), 'person_1_torso')
        self.assertEqual(person.left_hand.global_frame_id(), 'person_1_left_hand')
        self.assertEqual(person.right_hand.global_frame_id(), 'person_1_right_hand')
        self.assertEqual(person.default_body_part(), 'person_1_torso')
