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




class Robot(Entity):


    def __init__(self, config_file_path):
        Entity.__init__(self, 'robot')
        self.say_to_client = actionlib.SimpleActionClient('say_to', SayToAction)
        self.gesture_client = actionlib.SimpleActionClient('gesture', GestureAction)
        self.wait_for_action_servers(self.say_to_client, self.gesture_client)

        self.tts_duration_srv = rospy.ServiceProxy('tts_subsentence_duration', TextToSpeechSubsentenceDuration)
        self.wait_for_services(self.tts_duration_srv)

        self.robot_type = Robot.load_robot_type(config_file_path)
        self.gestures = Robot.load_gestures(config_file_path)
        self.facial_expressions = Robot.load_facial_expressions(config_file_path)
        self.event = None

    def wait(self, period):
        self.event = threading.Event()
        self.event.wait(timeout=period)

    def cancel_wait(self):
        if self.event is not None:
            self.event.set()

    def say_to(self, text, audience=None):
        if not isinstance(text, str):
                raise TypeError("say_to() parameter text={0} is not a str".format(text))

        if audience is not None and not isinstance(audience, Entity):
                raise TypeError("say_to() parameter audience={0} is not an Entity".format(audience))

        say_to_goal = SayToParser.parse(text, audience, self.tts_duration_srv)
        self.say_to_client.send_goal(say_to_goal)
        self.say_to_client.wait_for_result()

    def gaze_at(self, target):
        if isinstance(target, Entity):
            raise TypeError("gaze_at() parameter target={0} is not an Entity".format(target))

        gaze_goal = GazeGoal()
        gaze_goal.target = target

        self.gaze_client.send_goal(gaze_goal)
        self.gaze_client.wait_for_result()

    def gesture(self, type, duration, target=None):
        if not isinstance(type, str):
            raise TypeError("gesture() parameter type={0} is not a str".format(type))

        if not isinstance(duration, float):
            raise TypeError("gesture() parameter duration={0} is not a float".format(duration))

        if target is not None and not isinstance(target, Entity):
            raise TypeError("gesture() parameter target={0} is not an Entity".format(target))

        if type not in self.gestures:
            raise GestureDoesNotExistError("gesture type={0} does not exist and was not loaded from {1}'s config file".format(type, self.robot_type))

        gesture_goal = GestureGoal()
        gesture_goal.type = type
        gesture_goal.duration = duration

        if target is not None:
            gesture_goal.target = target

        self.gesture_client.send_goal(gesture_goal)
        self.gesture_client.wait_for_result()

    def facial_expression(self, type, duration):
        if not isinstance(type, str):
            raise TypeError("gesture() parameter type={0} is not a str".format(type))

        if not isinstance(duration, float):
            raise TypeError("gesture() parameter duration={0} is not a float".format(duration))

        if type not in self.gestures:
            raise FacialExpressionDoesNotExistError("facial expression type={0} does not exist and was not loaded from {1}'s config file".format(type, self.robot_type))

        fe_goal = FacialExpressionGoal()
        fe_goal.type = type
        fe_goal.duration = duration

        self.facial_client.send_goal(fe_goal)
        self.facial_client.wait_for_result()

    def do(self, *args):
        for i, action in enumerate(args):
            if not isinstance(action, (GestureGoal, GazeGoal, FacialExpressionGoal)):
                raise TypeError("do() parameter args[{0}]={1} is not a FacialExpressionGoal, GazeGoal or GestureGoal".format(i, action))

            if isinstance(action, GestureGoal):
                self.gesture_client.send_goal(action)
            elif isinstance(action, GazeGoal):
                pass
            elif isinstance(action, FacialExpressionGoal):
                pass

    def base_link(self):
        return "torso"

    @staticmethod
    def load_robot_type(config_file_path):
        with open(config_file_path, 'r') as file:
            config = yaml.load(file)
            robot_type = config['robot']['type']
            return robot_type

    @staticmethod
    def load_gestures(config_file_path):
        with open(config_file_path, 'r') as file:
            config = yaml.load(file)
            gestures = config['robot']['gestures']
            return gestures

    @staticmethod
    def load_facial_expressions(config_file_path):
        with open(config_file_path, 'r') as file:
            config = yaml.load(file)
            config = yaml.load(file)
            facial_expressions = config['robot']['facial_expressions']
            return facial_expressions
