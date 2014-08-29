from unittest import TestCase
from hri_msgs.msg import SayToGoal, GestureGoal
from hri_api.util import RobotConfigParser, SayToParser
from hri_api.util import GestureDoesNotExistError

__author__ = 'Jamie Diprose'

class Response(object):
    def __init__(self):
        self.duration = 1.0


def tts_duration_srv(sentence, start_i, end_i):
    return Response()


class TestSayToParser(TestCase):

    # def test_num_words(self):
    #     self.fail()
    #
    # def test_get_sentence(self):
    #     self.fail()

    def test_parse_say_to(self):
        valid_gestures = ['wave', 'point']
        goal_actual = SayToParser.parse_say_to("Hello I am a robot", 1, valid_gestures, tts_duration_srv)
        goal_expected = SayToGoal()
        goal_expected.text = "Hello I am a robot"
        goal_expected.gestures = []
        goal_expected.gesture_indicies = []
        goal_expected.audience_id = 1
        self.assertEqual(goal_actual, goal_expected)

        goal_actual_1 = SayToParser.parse_say_to("<wave>Hello I am a robot</wave>", 1, valid_gestures, tts_duration_srv)
        goal_expected_1 = SayToGoal()
        goal_expected_1.text = "Hello I am a robot"
        wave = GestureGoal()
        wave.type = 'wave'
        wave.duration = 1.0
        goal_expected_1.gestures = [wave]
        goal_expected_1.gesture_indicies = [0]
        goal_expected_1.audience_id = 1
        self.assertEqual(goal_actual_1, goal_expected_1)

        self.assertRaises(GestureDoesNotExistError, SayToParser.parse_say_to, "<nod>Hello I am a robot</nod>", 1, valid_gestures, tts_duration_srv)
        self.assertRaises(TypeError, SayToParser.parse_say_to, "<nod>Hello I am a robot</nod>", 'hello', valid_gestures, tts_duration_srv)
        self.assertRaises(TypeError, SayToParser.parse_say_to, "<nod>Hello I am a robot</nod>", 1, 1, tts_duration_srv)
        self.assertRaises(TypeError, SayToParser.parse_say_to, "<nod>Hello I am a robot</nod>", 1, valid_gestures, '1')

