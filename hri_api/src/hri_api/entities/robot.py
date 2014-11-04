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

from hri_msgs.msg import GestureAction, GestureGoal, ExpressionAction, TargetGoal, ExpressionGoal, TargetAction
from hri_msgs.srv import SayDuration
from hri_msgs.msg import SayActionFeedback, GazeActionFeedback, GestureActionFeedback, ExpressionActionFeedback
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
from collections import Callable
import inspect


class IActionHandle():
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        pass

    @abc.abstractmethod
    def cancel_action(self):
        """
        :return:
        """

    @abc.abstractmethod
    def wait_for_result(self):
        """

        :return:
        """

class SingleGoalActionHandle(IActionHandle):
    def __init__(self, action_client):
        IActionHandle.__init__(self)
        self.action_client = action_client

    def cancel_action(self):
        self.action_client.cancel_goal()

    def wait_for_result(self):
        self.action_client.wait_for_result()


class MultiGoalActionHandle(IActionHandle):
    def __init__(self, action_client, goal_handle):
        IActionHandle.__init__(self)
        self.action_client = action_client
        self.goal_handle = goal_handle

    def cancel_action(self):
        self.action_client.cancel_goal(self.goal_handle)

    def wait_for_result(self):
        self.action_client.wait_for_result(self.goal_handle)


class Default(object):
    """ Represents the default value for parameters in the gesture and expression class,

    """
    pass


class Robot(Entity):
    """ The Robot class represents a robot. It contains high level functions for representing and controlling robot
    actions, receiving callbacks about action events and subscribing to social communication.

    .. note::
        The Robot class is an abstract class and is never directly used to program a robot. Instead, each robot has
        its own class derived from Robot, e.g. the Nao robot is represented by the Nao class.

        See here for more details: http://en.wikipedia.org/wiki/Abstract_type

    """

    local_id = 0    # Id counter for robot classes
    SERVER_TIMEOUT = 5.0    # Timeout for connecting to action servers and services

    def __init__(self):
        self.local_id = Robot.local_id
        Robot.local_id += 1
        Entity.__init__(self, 'robot{0}'.format(self.local_id), None)

        robot_actions = Robot.read_supported_actions()
        robot_services = Robot.read_supported_services()

        self.say_found = False
        self.gaze_found = False
        self.gesture_found = False
        self.expression_found = False
        self.say_duration_found = False

        # Create the action and service proxies that connect to vendor specific action servers and services.
        if SayAction.__name__ in robot_actions:
            namespace = robot_actions[SayAction.__name__]['namespace']
            self.say_client = actionlib.SimpleActionClient(namespace, SayAction)
            self.say_found = Robot.wait_for_action_server(self.say_client, SayAction.__name__)
        else:
            self.say_client = None

        if GazeAction.__name__ in robot_actions:
            namespace = robot_actions[GazeAction.__name__]['namespace']
            self.gaze_client = actionlib.SimpleActionClient(namespace, TargetAction)
            self.gaze_found = Robot.wait_for_action_server(self.gaze_client, GazeAction.__name__)
        else:
            self.gaze_client = None

        if GestureAction.__name__ in robot_actions:
            namespace = robot_actions[GestureAction.__name__]['namespace']
            self.gesture_client = MultiGoalActionClient(namespace, GestureAction)
            self.gesture_found = Robot.wait_for_action_server(self.gesture_client, GestureAction.__name__)
        else:
            self.gesture_client = None

        if ExpressionAction.__name__ in robot_actions:
            namespace = robot_actions[Action.expression.name]['namespace']
            self.expression_client = MultiGoalActionClient(namespace, ExpressionAction)
            self.expression_found = Robot.wait_for_action_server(self.gesture_client, ExpressionAction.__name__)
        else:
            self.expression_client = None

        if SayDuration.__name__ in robot_services:
            name = robot_services[SayDuration.__name__]['name']
            self.say_duration_srv = rospy.ServiceProxy(namespace, SayDuration)
            self.say_duration_found = Robot.wait_for_service(self.say_duration_srv, SayDuration.__name__)
        else:
            self.say_duration_srv = None

        self.gaze_ah = None
        self.say_ah = None

        self.lock = threading.RLock()
        self.listen_cb = None
        self.listen_sub = None

    def default_body_part(self):
        raise NotImplementedError("Please implement this method")

    def register_listen_callback(self, callback):
        self.listen_cb = callback
        self.listen_sub = rospy.Subscriber("itf_listen", String, self.__listen, queue_size=10)

    def __listen(self, msg):

        if msg.data == 'BADINPUT':
            text = None
        else:
            text = msg.data

        self.listen_cb(text)

    def say(self, text, feedback_cb=None, done_cb=None):
        """ Say the text in the variable text.

        This function is asynchronous; it returns an ActionHandle, a unique identifier for the action being performed.
        An action handle can be used to wait for an action to complete or to stop an action

        :param text: The text the robot should say
        :type text: str

        :param feedback_cb: a function that will be called when the state of the say action changes. The function
        should have one parameter, which will be filled with a SayActionFeedback instance.
        :type feedback_cb: Callable

        :param done_cb: a function that will be called when the say action finishes.
        :type done_cb: Callable

        :return: an action handle to keep track of the action
        :rtype: SingleGoalActionHandle
        :raises TypeError: text is not a str, feedback_cb is not None or Callable, done_cb is not None or Callable
        """

        TypeChecker.accepts(inspect.currentframe().f_code.co_name,
                            (str, (None, Callable), (None, Callable)),
                            text, feedback_cb, done_cb)

        if Robot.is_action_runnable(self.say_client, SayAction.__name__, self.say_found):
            goal = SayGoal(text)
            self.say_client.send_goal(goal, feedback_cb, done_cb)
            ah = SingleGoalActionHandle(self.say_client)
            self.say_ah = ah
            return ah

    def say_and_wait(self, text, feedback_cb=None, done_cb=None):
        """ A synchronous version of the say function.

        The and_wait syntax signals that the function will not return until it completes.

        No ActionHandle is returned because the action completes before it returns.
        """

        TypeChecker.accepts(inspect.currentframe().f_code.co_name,
                            (str, (None, Callable), (None, Callable)),
                            text, feedback_cb, done_cb)

        self.say(text, feedback_cb, done_cb)
        self.say_client.wait_for_result()

    def gaze(self, target, speed=0.5, feedback_cb=None, done_cb=None):
        """ Makes the robot gaze at a target.

        This function is asynchronous and returns an SingleGoalActionHandle to keep track of the action.

        :param target: The target to gaze at
        :type target: Entity

        :param speed: The speed of the gaze action (normalised between 0.0 < speed <= 1.0)
        :type speed: float

        :param feedback_cb: a function that will be called when the state of the gaze action changes. The function
        should have one parameter, which will be filled with a GazeActionFeedback instance.
        :type feedback_cb: Callable

        :param done_cb: a function that will be called when the gaze action finishes.
        :type done_cb: Callable

        :return: an action handle to keep track of the action
        :rtype: SingleGoalActionHandle
        :raises TypeError: target is not an Entity, speed is not a float, feedback_cb is not None or Callable, done_cb
        is not None or Callable
        """

        TypeChecker.accepts(inspect.currentframe().f_code.co_name,
                            (Entity, float, (None, Callable), (None, Callable)),
                            target, speed, feedback_cb, done_cb)

        if Robot.is_action_runnable(self.gaze_client, GazeAction.__name__, self.gaze_found):
            World().add_to_world(target)
            goal = GazeGoal(target, speed, 0.3) #Todo override acceleration goal.acceleration = 0.3
            self.gaze_client.send_goal(goal, feedback_cb, done_cb)
            ah = SingleGoalActionHandle(self.gaze_client)
            self.gaze_ah = ah
            return ah

    def gaze_and_wait(self, target, speed=0.5, feedback_cb=None, done_cb=None):
        """ A synchronous version of the gaze function.

        No ActionHandle is returned because the action completes before it returns.
        """

        TypeChecker.accepts(inspect.currentframe().f_code.co_name,
                            (Entity, float, (None, Callable), (None, Callable)),
                            target, speed, feedback_cb, done_cb)

        self.gaze(target, speed, feedback_cb, done_cb)
        self.gaze_client.wait_for_result()

    def gesture(self, gesture, duration=Default(), target=None, feedback_cb=None, done_cb=None):
        """  Makes the robot perform a gesture, e.g. make a robot wave its left arm.

         The default values for the duration of each gesture are specified in the robots IGesture enumeration definition.

        :param gesture: the gesture to perform, an IGesture Python enumeration member e.g. for Nao, Gesture.WaveLArm.
        :type gesture: IGesture

        :param duration: the length of time the gesture plays for (seconds). If Default then the gesture is performed for
        its default duration. Use positive infinity for a never ending gesture, hint: float('inf')
        :type duration: float, Default

        :param target: the target to orient the gesture toward
        :type target: None, Entity

        :param feedback_cb: a function that will be called when the state of the gesture action changes. The function
        should have one parameter, which will be filled with a GestureActionFeedback instance.
        :type feedback_cb: Callable

        :param done_cb: a function that will be called when the gesture action finishes.
        :type done_cb: Callable

        :return: an action handle to keep track of the action
        :rtype: MultiGoalActionHandle
        :raises TypeError: gesture is not an IGesture member, duration is not a float or Default, target is not None or
        an Entity, feedback_cb is not None or Callable, done_cb is not None or Callable
        """

        TypeChecker.accepts(inspect.currentframe().f_code.co_name,
                            (IGesture, (Default, float), (None, Entity), (None, Callable), (None, Callable)),
                            gesture, duration, feedback_cb, done_cb)

        if Robot.is_action_runnable(self.gesture_client, GestureAction.__name__, self.gesture_found):
            if target is not None:
                World().add_to_world(target)

            goal = GestureGoal(gesture, duration, target) #TODO: check goal.target = target.get_id()
            gh = self.gesture_client.send_goal(goal, feedback_cb, done_cb)
            ah = MultiGoalActionHandle(self.gesture_client, gh)
            return ah

    def gesture_and_wait(self, gesture, duration=Default, target=None, feedback_cb=None, done_cb=None):
        """ A synchronous version of the gesture function.

        No ActionHandle is returned because the action completes before it returns.
        """

        TypeChecker.accepts(inspect.currentframe().f_code.co_name,
                            (IGesture, (Default, float), (None, Entity), (None, Callable), (None, Callable)),
                            gesture, duration, feedback_cb, done_cb)

        ah = self.gesture(gesture, duration, target, feedback_cb, done_cb)
        self.gesture_client.wait_for_result(ah.goal_handle)

    def expression(self, expression, intensity=Default(), speed=Default(), duration=Default(), feedback_cb=None, done_cb=None):
        """ Makes the robot perform a facial expression, e.g. smile.

        The default values for the intensity, speed and duration of each gesture are specified in the robots IExpression
        enumeration definition.

        This function is asynchronous and returns an ActionHandle.

        :param expression: The type of expression to perform, an IExpression enumeration member (e.g.for Zeno, Expression.Smile).
        :type expression: IExpression subclass member

        :param intensity: The strength of the expression e.g. a smile with an intensity of 1.0 is the largest possible smile
        (normalised between 0.0 < speed <= 1.0).
        :type intensity: float, Default

        :param speed: how fast the expression is performed, e.g. a smile expression with a speed of 1.0 is
        performed at the fastest possible speed (normalized between 0.0 <= speed <= 1.0). If Default() then the
        expression is performed at its default speed.
        :type speed: float, Default

        :param duration: how long the expression lasts (seconds). If Default() then the expression is performed for its
        default duration. Use positive infinity for a never ending expression, hint: float('inf')
        :type duration: float, Default

        :param feedback_cb: a function that will be called when the state of the expression action changes. The function
        should have one parameter, which will be filled with a ExpressionActionFeedback instance.
        :type feedback_cb: Callable

        :param done_cb: a function that will be called when the expression action finishes.
        :type done_cb: Callable

        :return: an action handle to keep track of the action
        :rtype: MultiGoalActionHandle
        :raises TypeError: expression is not an IExpression member, intensity is not a float or Default, speed is not a
        float or Default, duration is not a float or Default, feedback_cb is not None or Callable, done_cb is not None or Callable
        """

        TypeChecker.accepts(inspect.currentframe().f_code.co_name,
                            (IExpression, (Default, float), (Default, float), (Default, float), (None, Callable), (None, Callable)),
                            expression, intensity, speed, duration, feedback_cb, done_cb)

        if Robot.is_action_runnable(self.expression_client, ExpressionAction.__name__, self.expression_found):
            goal = ExpressionGoal(expression, intensity, speed, duration)
            gh = self.expression_client.send_goal(goal, feedback_cb, done_cb)
            ah = MultiGoalActionHandle(self.expression_client, gh)
            return ah

    def expression_and_wait(self, expression, intensity=Default, speed=Default, duration=Default, feedback_cb=None, done_cb=None):
        """ A synchronous version of the expression function.

        No ActionHandle is returned because the action completes before it returns.
        """

        TypeChecker.accepts(inspect.currentframe().f_code.co_name,
                            (IExpression, (Default, float), (Default, float), (Default, float), (None, Callable), (None, Callable)),
                            expression, intensity, speed, duration, feedback_cb, done_cb)

        ah = self.expression(expression, intensity, speed, duration, feedback_cb, done_cb)
        self.expression_client.wait_for_result(ah.goal_handle)

    def simultaneously(self, *goals):
        action_handles = []

        with self.lock:
            for goal in goals:
                if isinstance(goal, SayGoal):
                    if Robot.is_action_runnable(self.say_client, SayAction.__name__, self.say_found):
                        self.say_client.send_goal(goal)
                        ah = SingleGoalActionHandle(self.say_client)
                        action_handles.append(ah)

                elif isinstance(goal, GazeGoal):
                    if Robot.is_action_runnable(self.gaze_client, GazeAction.__name__, self.gaze_found):
                        self.gaze_client.send_goal(goal)
                        ah = SingleGoalActionHandle(self.gaze_client)
                        action_handles.append(ah)

                elif isinstance(goal, GestureGoal):
                    if Robot.is_action_runnable(self.gesture_client, GestureAction.__name__, self.gesture_found):
                        gh = self.gesture_client.send_goal(goal)
                        ah = MultiGoalActionHandle(self.gesture_client, gh)
                        action_handles.append(ah)

                elif isinstance(goal, ExpressionGoal):
                    if Robot.is_action_runnable(self.expression_client, ExpressionAction.__name__, self.expression_found):
                        gh = self.expression_client.send_goal(goal)
                        ah = MultiGoalActionHandle(self.expression_client, gh)
                        action_handles.append(ah)
                else:
                    raise TypeError('robot.simultaneously parameter goals has a goal with a unsupported type {0}, at index {1}. Should be one of: TextToSpeechGoal, TargetGoal, ExpressionGoal or GestureGoal'.format(type(goal), goals.index(goal)))

        return action_handles

    # TODO: simultaneously action handle
    # TODO: consecutively method and action handle

    # Wait for one or more goals to finish
    def wait(self, *action_handles):
        for ah in action_handles:
            #TODO: ParamAssertions.assert_types(self.wait, ah, IActionHandle)
            ah.wait_for_result()

    # Cancel one or more goals
    def cancel(self, *action_handles):
        for ah in action_handles:
            #TODO: ParamAssertions.assert_types(self.wait, ah, IActionHandle)
            ah.cancel_action()

    @staticmethod
    def is_action_runnable(action_client, action_type_name, action_found):
        if action_client is None:
            rospy.logerr("Cannot do {0} action because this action hasn't been configured for your robot. Please define "
                         "the {0} action and the namespace of its action server in your "
                         "robots yaml configuration file".format(action_type_name))
            return False

        elif not action_found:
            rospy.logerr("Cannot do {0} action because the {0} action server could not be found. Please check that "
                         "the {0} action server with the namespace {1} is running. "
                         "Hint: rostopic list | grep {1}".format(action_type_name, action_client.action_client.ns))
            return False

        return True

    @staticmethod
    def is_service_callable(service_proxy, service_type_name, service_found):
        if service_proxy is None:
            rospy.logerr("Cannot call {0} service because this service hasn't been defined for your robot. Please define "
                         "the {0} service and its name in your robots yaml configuration file".format(service_type_name))
            return False

        elif not service_found:
            rospy.logerr("Cannot call {0} service because the {0} service could not be found. Please check that "
                         "the {0} service with the name {1} is running. "
                         "Hint: rosservice list | grep {1}".format(service_type_name, service_proxy.name))
            return False

        return True

    @staticmethod
    def wait_for_action_server(action_client, action_type_name):
        connected = action_client.wait_for_server(timeout=rospy.Duration.from_sec(Robot.SERVER_TIMEOUT))

        if not connected:
            rospy.logerr("{0} actions are disabled because I could not connect to the {0} action server. Please "
                         "check that the {0} action server with the namespace {1} is running. "
                         "Hint: rostopic list | grep {1}".format(action_type_name, action_client.action_client.ns))

        return connected

    @staticmethod
    def wait_for_service(service_proxy, service_type_name):
        connected = service_proxy.wait_for_service(timeout=rospy.Duration.from_sec(Robot.SERVER_TIMEOUT))

        if not connected:
            rospy.logerr("The {0} service is disabled because I could not connect to the {0} service. Please "
                         "check that the {0} service with the namespace {1} is running. "
                         "Hint: rosservice list | grep {1}".format(service_type_name, service_proxy.name))

        return connected




