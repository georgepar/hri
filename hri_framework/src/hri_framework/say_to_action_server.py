#!/usr/bin/env python
import rospy
import actionlib
import abc
from hri_msgs.srv import TextToSpeechSubsentenceDuration, TextToSpeechSubsentenceDurationResponse
from hri_msgs.msg import TextToSpeechFeedback, TextToSpeechResult, TextToSpeechAction


class SayToActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.server = None
        self.feedback = None
        self.result = None