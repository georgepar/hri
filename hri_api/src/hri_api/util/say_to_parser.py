#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from hri_msgs.msg import SayToGoal, GestureGoal
from hri_api.query import is_callable
from .errors import GestureDoesNotExistError
import xml.etree.ElementTree as ET
import re


class SayToParser(object):
    valid_words_regex = "\w+[']{0,1}\w*[!?,.]{0,1}"
    punctuation = "[' ']"
    spaces = "[,.]"

    @staticmethod
    def num_words(text):
        return len(re.findall(SayToParser.valid_words_regex, text))

    @staticmethod
    def get_sentence(text):
        tree = ET.fromstring("<sayto>" + text + "</sayto>")
        text = ""

        for node in tree.iter():
            if node.tag == "sayto":
                if node.text is not None:
                    text += node.text + " "
            else:
                if node.text is not None:
                    text += node.text + " "

                if node.tail is not None:
                    text += node.text + " "

        return text.strip()




#
# class SayToGoalBuilder(object):
#     def __init__(self):
#         self.text = ""
#         self.gestures = []
#         self.gesture_start_words = {}
#         self.gesture_end_words = {}
#         self.audience = None
#
#
#
#     @staticmethod
#     def num_words(text):
#         return len(re.findall(SayTo.valid_words_regex, text))
#
#     def add_text(self, str text):
#         self.text += text
#
#     def add_gesture_start(self, GestureAt gesture):
#         self.gestures.append(gesture)
#         start_word = SayTo.num_words(self.text)
#         self.gesture_start_words[gesture] = start_word
#
#     def add_gesture_end(self, GestureAt gesture):
#         if gesture in self.gestures:
#             start_word = self.gesture_start_words[gesture]
#             end_word = SayTo.num_words(self.text)
#         else:
#             raise Exception("Gesture has not been added yet.")
#
#     def set_audience(self, Obj audience):
#         self.audience = audience
#
#     def get_goal(self):
#         goal = SayToGoal()
#         goal.text = self.text
#         goal.audience = self.audience.get_obj_id()
#         goal.gesture_goals = #Make gesture goals: includes their duration
#         goal.gesture_start_words = #For each gesture goal, find out its start word
#         return goal