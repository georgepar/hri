#!/usr/bin/env python
# Copyright (c) 2014, James Diprose
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from hri_msgs.msg import SayToAction, GestureAction, GestureGoal, ExpressionAction, GazeGoal, ExpressionGoal, GazeAction
from hri_msgs.srv import TextToSpeechSubsentenceDuration
import rospy
from .entity import Entity
from hri_api.entities import World
import actionlib
import threading
from hri_api.util import RobotConfigParser, SayToParser, GestureDoesNotExistError, FacialExpressionDoesNotExistError
from hri_api.actions import MultiGoalActionClient
from hri_api.entities import Speed, Intensity, Gesture, Expression
from hri_msgs.msg import TextToSpeechAction, TextToSpeechGoal
from hri_api.util import *
import abc
from hri_api.query import Query
from actionlib import ClientGoalHandle
import random


class Robot(Entity):
    ENTITY_TYPE = 'robot'

    def __init__(self, expression_enum, gesture_enum):
        Entity.__init__(self, Robot.ENTITY_TYPE, Robot.ENTITY_TYPE + str(1), None)
        ParamFormatting.assert_types(self.__init__, expression_enum, Expression)
        ParamFormatting.assert_types(self.__init__, gesture_enum, Gesture)

        self.expression_enum = expression_enum
        self.gesture_enum = gesture_enum

        self.expression_client = MultiGoalActionClient('expression', ExpressionAction)
        self.gaze_client = actionlib.SimpleActionClient('gaze', GazeAction)
        self.tts_client = actionlib.SimpleActionClient('text_to_speech', TextToSpeechAction)
        self.gesture_client = MultiGoalActionClient('gesture', GestureAction)
        self.wait_for_action_servers(self.gaze_client, self.expression_client, self.tts_client, self.gesture_client)

        self.tts_duration_srv = rospy.ServiceProxy('tts_subsentence_duration', TextToSpeechSubsentenceDuration)
        self.wait_for_services(self.tts_duration_srv)

        self.event = None

        # Goal handles
        self.say_gh = None
        self.gaze_gh = None
        self.multi_action_clients = {}

        #Say to
        self.say_to_goal_handles = []
        self.audience = None

    # Text to speech
    def say(self, text):
        ParamFormatting.assert_types(self.say, text, str)
        goal = TextToSpeechGoal()
        goal.sentence = text
        self.tts_client.send_goal(goal, feedback_cb=self.say_feedback, done_cb=self.say_done)
        self.say_gh = self.tts_client.action_client.gh
        return self.say_gh

    def say_and_wait(self, text):
        self.say(text)
        self.tts_client.wait_for_result()

    def say_done(self, state, result):
        self.say_gh = None

    # Gaze
    def gaze(self, target, speed=0.5):
        ParamFormatting.assert_types(self.gaze, target, Entity)
        ParamFormatting.assert_types(self.gaze, speed, float)
        ParamFormatting.assert_range(self.gaze, speed, 0.0, 1.0)

        World().add_to_world(target)
        goal = GazeGoal()
        goal.target = target.get_id()
        goal.speed = speed
        goal.acceleration = 0.3

        self.gaze_client.send_goal(goal, feedback_cb=self.gaze_feedback, done_cb=self.gaze_done)
        self.gaze_gh = self.gaze_client.action_client.gh
        return self.gaze_gh

    def gaze_and_wait(self, target, speed=0.5, timeout=rospy.Duration()):
        self.gaze(target, speed)
        self.gaze_client.wait_for_result(timeout)

    def gaze_feedback(self, feedback):
        pass

    def gaze_done(self, state, result):
        self.gaze_gh = None

    # Blinking
    def blink(self, blink_duration, blink_rate_mean, blink_rate_sd):
        """
        :param blink_duration:
        :param blink_rate_mean:
        :param blink_rate_sd:
        :return:
        """
        raise NotImplementedError("Please implement the blink method")

    # Facial expressions
    def expression(self, expression, intensity=0.5, speed=0.5, duration=rospy.Duration()):
        ParamFormatting.assert_types(self.expression, expression, Expression)

        ParamFormatting.assert_types(self.expression, intensity, float)
        ParamFormatting.assert_range(self.expression, intensity, 0.0, 1.0)

        ParamFormatting.assert_types(self.expression, speed, float)
        ParamFormatting.assert_range(self.expression, speed, 0.0, 1.0)

        ParamFormatting.assert_types(self.expression, duration, rospy.Duration)

        goal = ExpressionGoal()
        goal.expression = expression.name
        goal.intensity = intensity
        goal.speed = speed
        goal.duration = duration

        gh = self.expression_client.send_goal(goal, done_cb=self.expression_done)
        self.multi_action_clients[gh] = gh
        return gh

    def expression_and_wait(self, expression, intensity=0.5, speed=0.5, duration=rospy.Duration(), timeout=rospy.Duration()):
        gh = self.expression(expression, intensity, speed, duration)
        self.expression_client.wait_for_result(gh, timeout)

    def expression_done(self, goal_handle):
        self.multi_action_clients.pop(goal_handle)

    # Gestures
    def gesture(self, gesture, target, intensity=0.5, speed=0.5, duration=rospy.Duration()):
        ParamFormatting.assert_types(self.gesture, gesture, Gesture)
        ParamFormatting.assert_types(self.gesture, target, Entity, Query)

        ParamFormatting.assert_types(self.gesture, intensity, float)
        ParamFormatting.assert_range(self.gesture, intensity, 0.0, 1.0)

        ParamFormatting.assert_types(self.gesture, speed, float)
        ParamFormatting.assert_range(self.gesture, speed, 0.0, 1.0)

        ParamFormatting.assert_types(self.expression, duration, rospy.Duration)

        World().add_to_world(target)
        goal = GestureGoal()
        goal.gesture = gesture.name
        goal.target = target.get_id()
        goal.duration = duration

        gh = self.gesture_client.send_goal(goal, done_cb=self.gesture_done)
        self.multi_action_clients[gh] = gh
        return gh

    def gesture_and_wait(self, gesture, intensity=0.5, speed=0.5, duration=rospy.Duration(), timeout=rospy.Duration()):
        gh = self.gesture(gesture, intensity, speed, duration)
        self.gesture_client.wait_for_result(gh, timeout)

    def gesture_done(self, goal_handle):
        self.multi_action_clients.pop(goal_handle)

    # Speaking, gazing and gesturing simultaneously
    def say_to_and_wait(self, text, audience):
        ParamFormatting.assert_types(self.say_to, text, str)
        ParamFormatting.assert_types(self.say_to, audience, Entity, Query)
        (sentence, ) = SayToParser.parse_say_to(text, audience, self.expression_enum, self.gesture_enum)

        # Pick a person to gaze at initially
        if isinstance(audience, Entity):
            person = audience
            self.gaze_and_wait(person)
        else:
            results = audience.order_by_ascending(lambda p: p.distance_to(self)).execute()

            if len(results) > 0:
                person = results[0]
                self.gaze_and_wait(person)

        self.say_and_wait(text)
        self.wait(*self.say_to_goal_handles)

    def say_feedback(self, feedback):
        if feedback.current_word_num in self.change_gaze:
            if isinstance(self.audience, Entity):
                self.gaze(self.audience)
            else:
                people = self.audience.execute()

                if len(people) > 1:
                    person = random.choice(self.people - self.cur_gazee)
                    self.gaze(person)
                elif len(people) == 1:
                    self.gaze(people[0])

        if feedback.current_word_num in self.say_to_gestures:
            gesture = self.say_to_gestures[feedback.current_word_num]
            gh = self.do(gesture)
            self.say_to_goal_handles.append(gh)

        if feedback.current_word_num in self.say_to_expressions:
            expression = self.say_to_expressions[feedback.current_word_num]
            gh = self.do(expression)
            self.say_to_goal_handles.append(gh)

    def do(self, *goals):
        goal_handles = []

        for goal in goals:
            ParamFormatting.assert_types(self.do, goal, [GazeGoal, ExpressionGoal, GestureGoal, TextToSpeechGoal])

            if isinstance(goal, TextToSpeechGoal):
                self.tts_client.send_goal()
                goal_handles.append() #TODO: get goal handle and append it

            elif isinstance(goal, GazeGoal):
                self.gaze_client.send_goal(goal)
                goal_handles.append() #TODO: get goal handle and append it

            elif isinstance(goal, ExpressionGoal):
                gh = self.expression_client.send_goal(goal)
                goal_handles.append(gh)

            elif isinstance(goal, GestureGoal):
                gh = self.gesture_client.send_goal(goal)
                goal_handles.append(gh)

        return goal_handles

    # Wait for one or more goals to finish
    def wait(self, *goal_handles):
        for goal_handle in goal_handles:
            if goal_handle == self.gaze_gh:
                self.gaze_client.wait_for_result()
            elif goal_handle == self.say_gh:
                self.tts_client.wait_for_result()
            elif goal_handle in self.multi_action_clients:
                client = self.multi_action_clients[goal_handle]
                client.wait_for_result(goal_handle)

    # Cancel one or more goals
    def cancel(self, *goal_handles):
        for goal_handle in goal_handles:
            ParamFormatting.assert_types(self.cancel, goal_handle, ClientGoalHandle)

            if goal_handle == self.gaze_gh:
                self.gaze_client.cancel_goal()
            elif goal_handle == self.say_gh:
                self.tts_client.cancel_goal()
            elif goal_handle in self.multi_action_clients:
                client = self.multi_action_clients[goal_handle]
                client.cancel_goal(goal_handle)

    def default_tf_frame_id(self):
        raise NotImplementedError("Please implement this method")

    def tf_frame_id(self):
        raise NotImplementedError("Please implement this method")

    # Wait for a period of time
    def wait_for_period(self, period):
        self.event = threading.Event()
        self.event.wait(timeout=period)

    def cancel_wait_for_period(self):
        if self.event is not None:
            self.event.set()




