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

from hri_msgs.msg import GestureAction, GestureGoal, ExpressionAction, TargetGoal, TargetAction
from hri_msgs.msg import ExpressionGoal as IExpressionGoal
from hri_msgs.srv import TextToSpeechSubsentenceDuration
import rospy
from .entity import Entity
from hri_api.entities import World
import actionlib
import threading
from hri_api.actions import MultiGoalActionClient
from hri_api.entities import IGesture, IExpression
from hri_msgs.msg import TextToSpeechAction, TextToSpeechGoal
from hri_api.util import *
import abc
from hri_api.query import Query
from actionlib import ClientGoalHandle
import random
from enum import Enum
import re
import xml.etree.ElementTree as ET
from threading import Thread
from std_msgs.msg import String
import weakref


class Robot(Entity):
    """ The Robot class represents a robot and contains high level functions for controlling robot actions and
    subscribing to social communication.

    .. note::
        The Robot class is an abstract class and is never directly used to program a robot. Instead, each robot has
        its own class derived from Robot, e.g. the Nao robot is represented by the Nao class.

        See here for more details: http://en.wikipedia.org/wiki/Abstract_type

    """

    __metaclass__ = abc.ABCMeta

    def __init__(self, expressions=None, gestures=None):

        Entity.__init__(self, 'robot' + str(1), None)
        self.expressions = expressions
        self.gestures = gestures

        # Create the action and service proxies that connect to robot specific action servers and services.
        #TODO: read topics from a config file, if not found then that action / service wont be used


        self.say_client = actionlib.SimpleActionClient('text_to_speech', SayAction)
        self.say_done_cbm = CallbackManager()
        self.say_feedback_cbm = CallbackManager()

        self.gaze_client = actionlib.SimpleActionClient('gaze', TargetAction)
        self.gaze_done_cbm = CallbackManager()
        self.gaze_feedback_cbm = CallbackManager()

        self.expression_client = MultiGoalActionClient('expression', ExpressionAction)
        self.expression_done_cbm = CallbackManager()
        self.expression_feedback_cbm = CallbackManager()

        self.gesture_client = MultiGoalActionClient('gesture', GestureAction)
        self.gesture_done_cbm = CallbackManager()
        self.gesture_feedback_cbm = CallbackManager()

        self.say_duration_srv = rospy.ServiceProxy('say_duration', SayDuration)

        self.event = None
        self.action_handles = []
        self.gaze_ah = None
        self.say_ah = None

        self.lock = threading.RLock()
        self.listen_cb = None
        self.listen_sub = None

        self.gesture_done_cbm = CallbackManager()

    def register_listen_callback(self, callback):
        self.listen_cb = callback
        self.listen_sub = rospy.Subscriber("itf_listen", String, self.__listen, queue_size=10)

    def __listen(self, msg):

        if msg.data == 'BADINPUT':
            text = None
        else:
            text = msg.data

        self.listen_cb(text)

    # Text to speech
    def say(self, text):
        """ Synthesise the text in the string text. This function is asynchronous; it returns an ActionHandle, a
        unique identifier for the action being performed. An action handle can be used to wait for an action to complete
        or to cancel an action

        :param text: The text the robot should say
        :type text: str
        :return: an action handle to keep track of the action
        :rtype: SingleGoalActionHandle
        :raises TypeError: text is not a str
        """

        ParamFormatting.assert_types(self.say, text, str)
        goal = TextToSpeechGoal()
        goal.sentence = text
        self.say_client.send_goal(goal, feedback_cb=self.__say_feedback, done_cb=self.__say_done)
        ah = SingleGoalActionHandle(self.say_client)
        self.say_ah = ah
        self.add_action_handle(ah)
        return ah

    def say_and_wait(self, text):
        """ A synchronous version of the say function. The and_wait syntax signals that the function will not return
        until it completes. No ActionHandle is returned because the action completes before it returns.

        :param text: The text the robot should say
        :type text: str
        """

        self.say(text)
        self.say_client.wait_for_result()

    def __say_done(self, state, result):
        self.say_done_cbm.execute()

        with self.lock:
            self.remove_action_handle(self.say_ah)
            self.say_ah = None

    def add_say_done_callback(self, callback):
        """Register a callback, it will be fired when the say action finishes

        :param callback: The callback to add
        :type callback: callable
        """

        self.say_done_cbm.add_callback(callback)

    def remove_say_done_callback(self, callback):
        """Remove a callback that was registered to fire when the say action finished

        :param callback: The callback to remove
        :type callback: callable
        """
        self.say_done_cbm.remove_callback(callback)

    def __say_feedback(self, feedback):
        self.say_feedback_cbm.execute(feedback.current_word_index)

    def register_say_feedback_callback(self, callback):
        """Register a callback, it will be fired when feedback is generated from the say action

        :param callback: The callback to add
        :type callback: callable
        """

        self.say_feedback_cbm.add_callback(callback)

    def remove_say_feedback_callback(self, callback):
        """Remove a callback that was registered to fire when feedback was generated from the say action

        :param callback: The callback to remove
        :type callback: callable
        """

        self.say_feedback_cbm.remove_callback(callback)

    def gaze(self, target, speed=0.5):
        """ Makes the robot gaze at a target. The speed that the robot gazes at is controlled by the speed parameter.
        This function is asynchronous and returns an ActionHandle to keep track of the action.

        :param target: The target to gaze at
        :type target: Entity
        :param speed: The speed of the gaze action (normalised between 0.0 < speed <= 1.0)
        :type speed: float
        :return: an action handle to keep track of the action
        :rtype: SingleGoalActionHandle
        :raises TypeError: target is not an Entity, speed is not a float or speed is not within the range 0.0 < speed <= 1.0
        """

        ParamFormatting.assert_types(self.gaze, target, Entity)
        ParamFormatting.assert_types(self.gaze, speed, float)
        ParamFormatting.assert_range(self.gaze, speed, 0.0, 1.0)

        World().add_to_world(target)
        goal = TargetGoal()
        goal.target = target.global_id()
        goal.speed = speed
        goal.acceleration = 0.3

        self.gaze_client.send_goal(goal, feedback_cb=self.__gaze_feedback, done_cb=self.__gaze_done)
        ah = SingleGoalActionHandle(self.gaze_client)
        self.gaze_ah = ah
        self.add_action_handle(ah)
        return ah

    def gaze_and_wait(self, target, speed=0.5, timeout=rospy.Duration()):
        """ A synchronous version of the gaze function. No ActionHandle is returned because the action completes
        before it returns.

        :param target: The target to gaze at
        :type target: Entity
        :param speed: The speed of the gaze action (normalised between 0.0 < speed <= 1.0)
        :type speed: float
        :param timeout: the duration to gaze before exiting
        :type timeout: rospy.Duration
        """

        self.gaze(target, speed)
        self.gaze_client.wait_for_result(timeout)

    def __gaze_feedback(self, feedback):
        self.gaze_feedback_cbm.execute(feedback.distance_to_target)

    def register_gaze_feedback_callback(self, callback):
        """Register a callback, it will be fired when feedback is generated from the gaze action

        :param callback: The callback to add
        :type callback: callable
        """

        self.gaze_feedback_cbm.add_callback(callback)

    def remove_gaze_feedback_callback(self, callback):
        """Remove a callback that was registered to fire when feedback was generated from the gaze action

        :param callback: The callback to remove
        :type callback: callable
        """

        self.gaze_feedback_cbm.remove_callback(callback)

    def __gaze_done(self, state, result):
        self.gaze_done_cbm.execute()
        self.gaze_ah = None

    def register_gaze_done_callback(self, callback):
        """Register a callback, it will be fired when the  gaze action finishes

        :param callback: The callback to add
        :type callback: callable
        """
        self.gaze_done_cbm.add_callback(callback)

    def remove_gaze_done_callback(self, callback):
        """Remove a callback that was registered to fire when the gaze action finished

        :param callback: The callback to remove
        :type callback: callable
        """

        self.gaze_done_cbm.remove_callback(callback)

    def expression(self, expression, intensity=Default, speed=Default, duration=Default):
        """ Makes the robot perform a facial expression, e.g. smile. The type of expression is specified by the
        expression parameter, an IExpression subclass member (e.g.for Zeno, Expression.Smile).

        :param expression: The expression to perform
        :type expression: IExpression subclass member
        :param intensity: The intensity of the expression (normalised between 0.0 < speed <= 1.0)
        :type intensity: float, Default
        :param speed: The speed of the expression (normalised between 0.0 < speed <= 1.0)
        :type speed: float, Default
        :param duration: The duration of the expression (normalised between 0.0 < speed <= 1.0)
        :type duration: float, Default
        :return: an action handle to keep track of the action
        :rtype: MultiGoalActionHandle
        :raises TypeError: target is not an Entity, speed is not a float or speed is not within the range 0.0 < speed <= 1.0
        """

        ParamFormatting.assert_types(self.expression, expression, IExpression)

        if intensity is not Default:
            ParamFormatting.assert_types(self.expression, intensity, float)
            ParamFormatting.assert_range(self.expression, intensity, 0.0, 1.0)

        if speed is not Default:
            ParamFormatting.assert_types(self.expression, speed, float)
            ParamFormatting.assert_range(self.expression, speed, 0.0, 1.0)

        if duration is not Default:
            ParamFormatting.assert_types(self.expression, duration, float)
            ParamFormatting.assert_greater_than(self.expression, duration, 0.0)

        goal = ExpressionGoal(expression, intensity, speed, duration)
        gh = self.expression_client.send_goal(goal, done_cb=self.__expression_done)
        ah = MultiGoalActionHandle(self.expression_client, gh)
        self.add_action_handle(ah)
        return ah

    def expression_and_wait(self, expression, intensity=None, speed=None, duration=None, timeout=rospy.Duration()):
        ah = self.expression(expression, intensity, speed, duration)
        self.expression_client.wait_for_result(ah.goal_handle, timeout)

    def __expression_done(self, goal_handle):
        with self.lock:
            ah = self.get_action_handle(goal_handle)
            self.remove_action_handle(ah)

    def register_expression_done_callback(self, callback):
        """Register a callback, it will be fired when the expression action finishes

        :param callback: The callback to add
        :type callback: callable
        """
        self.expression_done_cbm.add_callback(callback)

    def remove_expression_done_callback(self, callback):
        """Remove a callback that was registered to fire when the expression action finished

        :param callback: The callback to remove
        :type callback: callable
        """

        self.expression_done_cbm.remove_callback(callback)

    # Gestures
    def gesture(self, gesture, target=None, duration=None):

        ParamFormatting.assert_types(self.gesture, gesture, IGesture)

        goal = GestureGoal()
        goal.gesture = gesture.name

        if target is None:
            goal.target = ''
        else:
            World().add_to_world(target)
            ParamFormatting.assert_types(self.gesture, target, Entity)
            goal.target = target.global_id()

        if duration is None:
            goal.duration = -1
        else:
            ParamFormatting.assert_types(self.expression, duration, float)
            ParamFormatting.assert_greater_than(self.expression, duration, 0.0)
            goal.duration = duration

        gh = self.gesture_client.send_goal(goal, done_cb=self.__gesture_done)

        ah = MultiGoalActionHandle(self.gesture_client, gh)
        self.add_action_handle(ah)
        return ah

    def gesture_and_wait(self, gesture, target=None, duration=None, timeout=rospy.Duration()):
        ah = self.gesture(gesture, target, duration)
        self.gesture_client.wait_for_result(ah.goal_handle, timeout)

    def __gesture_done(self, goal_handle):
        with self.lock:
            ah = self.get_action_handle(goal_handle)
            self.remove_action_handle(ah)

    def add_action_handle(self, action_handle):
        with self.lock:
            ParamFormatting.assert_types(self.add_action_handle, action_handle, IActionHandle)
            self.action_handles.append(action_handle)

    def get_action_handle(self, goal_handle):
        with self.lock:
            for ah in self.action_handles:
                if isinstance(ah, MultiGoalActionHandle):
                    if ah.goal_handle == goal_handle:
                        return ah

    def remove_action_handle(self, action_handle):
        with self.lock:
            ParamFormatting.assert_types(self.remove_action_handle, action_handle, IActionHandle)
            if action_handle in self.action_handles:
                    self.action_handles.remove(action_handle)

    def simultaneously(self, *goals):
        action_handles = []

        with self.lock:
            for goal in goals:
                if isinstance(goal, SayGoal):
                    self.say_client.send_goal(goal)
                    ah = SingleGoalActionHandle(self.say_client)
                    self.add_action_handle(ah)
                    action_handles.append(ah)

                elif isinstance(goal, GazeGoal):
                    self.gaze_client.send_goal(goal)
                    ah = SingleGoalActionHandle(self.gaze_client)
                    self.add_action_handle(ah)
                    action_handles.append(ah)

                elif isinstance(goal, ExpressionGoal):
                    gh = self.expression_client.send_goal(goal)
                    ah = MultiGoalActionHandle(self.expression_client, gh)
                    self.add_action_handle(ah)
                    action_handles.append(ah)

                elif isinstance(goal, GestureGoal):
                    gh = self.gesture_client.send_goal(goal)
                    ah = MultiGoalActionHandle(self.gesture_client, gh)
                    self.add_action_handle(ah)
                    action_handles.append(ah)
                else:
                    raise TypeError('robot.simultaneously parameter goals has a goal with a unsupported type {0}, at index {1}. Should be one of: TextToSpeechGoal, TargetGoal, ExpressionGoal or GestureGoal'.format(type(goal), goals.index(goal)))

        return action_handles

    def consecutively(self, *goals):
        with self.lock:
            pass

    # Wait for one or more goals to finish
    def wait(self, *action_handles):
        for ah in action_handles:
            ParamFormatting.assert_types(self.wait, ah, IActionHandle)
            ah.wait_for_result()
            self.remove_action_handle(action_handle=ah)

    # Cancel one or more goals
    def cancel(self, *action_handles):
        for ah in action_handles:
            ParamFormatting.assert_types(self.wait, ah, IActionHandle)
            ah.cancel_action()
            self.remove_action_handle(action_handle=ah)
