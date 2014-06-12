#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from hri_msgs.msg import SayToAction, GestureAction, SayToGoal, GestureGoal, UUID, FacialExpressionGoal, GazeGoal
from hri_msgs.srv import TextToSpeechSubsentenceDuration
import rospy
from .entity import Entity
import actionlib
import threading
from xml.etree.ElementTree import ElementTree
import re
import yaml

class SayToParser(object):
    valid_words_regex = "\w+[']{0,1}\w*[!?,.]{0,1}"
    punctuation = "[' ']"
    spaces = "[,.]"

    def __init__(self, text, audience, tts_duration_srv):
         @staticmethod

    def num_words(text):
        return len(re.findall(SayToParser.valid_words_regex, text))

    @staticmethod
    def get_sentence(text):
        tree = ElementTree.fromstring("<sayto>" + text + "</sayto>")
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

        return text

    def say_to(self, text, audience=None):
        if not isinstance(text, str):
                raise TypeError("say_to() parameter text={0} is not a str".format(text))

        if audience is not None and not isinstance(audience, Entity):
                raise TypeError("say_to() parameter audience={0} is not an AbstractEntity".format(audience))

        say_to_goal = SayToGoal()

        if audience is not None:
            say_to_goal.audience_id = id(audience)

        xml_tree = ElementTree.fromstring("<sayto>" + text + "</sayto>")
        say_to_goal.text = Robot.get_sentence(text)
        seen_text = ''

        for node in xml_tree.iter():
            if node.tag == "sayto":
                if node.text is not None:
                    seen_text += node.text + " "
            else:
                start_word_i = Robot.num_words(seen_text)

                if node.text is not None:
                    seen_text += node.text + " "

                end_word_i = Robot.num_words(seen_text)

                gesture_type = node.tag

                if gesture_type not in self.gestures:
                    raise GestureDoesNotExistError("gesture={0} does not exist and was not loaded from {1}'s config file".format(gesture_type, self.robot_type))

                gesture_goal = GestureGoal()
                gesture_goal.type = gesture_type
                gesture_goal.duration = self.tts_duration_srv(say_to_goal.text, start_word_i, end_word_i).duration

                if "target" in node.attrib:
                    uuid = UUID()
                    uuid.uuid = node.attrib["target"]
                    gesture_goal.target = uuid

                say_to_goal.gestures.append(gesture_goal)
                say_to_goal.gesture_indicies.append(start_word_i)

                if node.tail is not None:
                    text += node.tail + " "

        self.say_to_client.send_goal(say_to_goal)
        self.say_to_client.wait_for_result()

class SayToGoalBuilder(object):
    def __init__(self):
        self.text = ""
        self.gestures = []
        self.gesture_start_words = {}
        self.gesture_end_words = {}
        self.audience = None



    @staticmethod
    def num_words(text):
        return len(re.findall(SayTo.valid_words_regex, text))

    def add_text(self, str text):
        self.text += text

    def add_gesture_start(self, GestureAt gesture):
        self.gestures.append(gesture)
        start_word = SayTo.num_words(self.text)
        self.gesture_start_words[gesture] = start_word

    def add_gesture_end(self, GestureAt gesture):
        if gesture in self.gestures:
            start_word = self.gesture_start_words[gesture]
            end_word = SayTo.num_words(self.text)
        else:
            raise Exception("Gesture has not been added yet.")

    def set_audience(self, Obj audience):
        self.audience = audience

    def get_goal(self):
        goal = SayToGoal()
        goal.text = self.text
        goal.audience = self.audience.get_obj_id()
        goal.gesture_goals = #Make gesture goals: includes their duration
        goal.gesture_start_words = #For each gesture goal, find out its start word
        return goal