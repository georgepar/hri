#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
from hri_msgs.msg import SayToAction, GestureAction, GestureGoal, FacialExpressionGoal, GazeGoal, FacialExpressionAction
from hri_msgs.srv import TextToSpeechSubsentenceDuration
import rospy
from .entity import Entity
from hri_api.entities import World
import actionlib
import threading
from hri_api.util import RobotConfigParser, SayToParser, GestureDoesNotExistError, FacialExpressionDoesNotExistError


class Robot(Entity):

    def __init__(self, robot_type, robot_id, config_file):
        Entity.__init__(self, robot_type, robot_id, None)
        self.say_to_client = actionlib.SimpleActionClient('say_to', SayToAction)
        self.gesture_client = actionlib.SimpleActionClient('gesture', GestureAction)
        self.facial_expression_client = actionlib.SimpleActionClient('facial_expression', FacialExpressionAction)
        self.wait_for_action_servers(self.say_to_client, self.gesture_client, self.facial_expression_client)

        self.tts_duration_srv = rospy.ServiceProxy('tts_subsentence_duration', TextToSpeechSubsentenceDuration)
        self.wait_for_services(self.tts_duration_srv)

        self.robot_type = RobotConfigParser.load_robot_type(config_file)
        self.gestures = RobotConfigParser.load_gestures(config_file)
        self.facial_expressions = RobotConfigParser.load_facial_expressions(config_file)
        self.event = None

        self.actions_lock = threading.RLock()
        self.actions = {}

    def __add_action(self, action_handle):
        with self.actions_lock:
            if action_handle.namespace not in self.actions:
                self.actions[action_handle.namespace] = []

            sub_actions = self.actions[action_handle.namespace]
            sub_actions.append(action_handle)

    def __remove_action(self, action_handle):
        with self.actions_lock:
            if action_handle.namespace not in self.actions:
                self.actions[action_handle.namespace] = []

            sub_actions = self.actions[action_handle.namespace]
            sub_actions.remove(action_handle)

    def cancel_action(self, action_handle):
        action_handle.cancel()
        self.__remove_action(action_handle)

    def __cancel_actions_of_namespace(self, namespace):
        with self.actions_lock:
            if namespace not in self.actions:
                self.actions[namespace] = []

            sub_action_handles = self.actions[namespace]

            for action_handle in sub_action_handles:
                self.cancel_action(action_handle)

    def wait_for_action(self, action_handle, timeout_period=None):
        if not isinstance(action_handle, ActionHandle):
            raise TypeError("wait_for_action() parameter action_handle={0} is not an ActionHandle".format(action_handle))

        if timeout_period is not None:
            if not isinstance(timeout_period, float):
                raise TypeError("wait_for_action() parameter timeout_period={0} is not a float".format(timeout_period))

            result = self.action.action_server.wait_for_result(rospy.Duration(timeout_period))
        else:
            result = self.action.action_server.wait_for_result()
            self.__remove_action(action_handle)

        return result

    def say_to(self, text, audience=None):
        if not isinstance(text, str):
            raise TypeError("say_to() parameter text={0} is not a str".format(text))

        if audience is not None and not isinstance(audience, Entity):
            raise TypeError("say_to() parameter audience={0} is not an Entity".format(audience))

        if audience is not None:
            World().add_to_world(audience)

        self.__cancel_actions_of_namespace(self.say_to_client.action_client.ns)

        say_to_goal = SayToParser.parse(text, audience, self.gestures, self.tts_duration_srv)
        self.say_to_client.send_goal(say_to_goal)
        action_handle = ActionHandle(self.say_to_client, self.say_to_client.gh.comm_state_machine.action_goal.goal_id)
        self.__add_action(action_handle)
        return action_handle

    def gaze_at(self, target):
        if not isinstance(target, Entity):
            raise TypeError("gaze_at() parameter target={0} is not an Entity".format(target))

        self.cancel_actions_of_namespace(self.say_to_client.gaze_client.ns)

        World().add_to_world(target)
        goal = GazeGoal()
        goal.target = target

        self.gaze_client.send_goal(goal)
        self.gaze_client.wait_for_result()
        action_handle = ActionHandle(self.gaze_client, self.say_to_client.gh.comm_state_machine.action_goal.goal_id)
        self.__add_action(action_handle)
        return action_handle

        return ActionHandle(self.gaze_client, id(goal), goal)

    def gesture_at(self, type, duration, target=None):
        if not isinstance(type, str):
            raise TypeError("gesture() parameter type={0} is not a str".format(type))

        if not isinstance(duration, float):
            raise TypeError("gesture() parameter duration={0} is not a float".format(duration))

        if target is not None and not isinstance(target, Entity):
            raise TypeError("gesture() parameter target={0} is not an Entity".format(target))

        if type not in self.gestures:
            raise GestureDoesNotExistError("gesture type={0} does not exist and was not loaded from {1}'s config file".format(type, self.robot_type))

        self.cancel_actions_of_namespace(self.say_to_client.gesture_client.ns)

        goal = GestureGoal()
        goal.type = type
        goal.duration = duration

        if target is not None:
            World().add_to_world(target)
            goal.target = target

        self.gesture_client.send_goal(goal)
        action_handle = ActionHandle(self.gesture_client, self.gesture_client.gh.comm_state_machine.action_goal.goal_id)
        self.__add_action(action_handle)
        return action_handle

    def show_expression(self, type, duration, intensity):
        if not isinstance(type, str):
            raise TypeError("show_expression() parameter type={0} is not a str".format(type))

        if not isinstance(duration, float):
            raise TypeError("show_expression() parameter duration={0} is not a float".format(duration))

        if not isinstance(duration, float):
            raise TypeError("show_expression() parameter intensity={0} is not a float".format(duration))

        if type not in self.facial_expressions:
            raise FacialExpressionDoesNotExistError("facial expression type={0} does not exist and was not loaded from {1}'s config file".format(type, self.robot_type))

        self.cancel_actions_of_namespace(self.say_to_client.facial_expression_client.ns)

        goal = FacialExpressionGoal()
        goal.type = type
        goal.duration = duration
        goal.intensity = intensity

        self.facial_expression_client.send_goal(goal)
        action_handle = ActionHandle(self.facial_expression_client, self.facial_expression_client.gh.comm_state_machine.action_goal.goal_id)
        self.__add_action(action_handle)
        return action_handle

    def default_tf_frame_id(self):
        raise NotImplementedError("Please implement this method")

    def tf_frame_id(self):
        raise NotImplementedError("Please implement this method")

    # def wait_for_period(self, period):
    #     self.event = threading.Event()
    #     self.event.wait(timeout=period)
    #
    # def cancel_wait_for_period(self):
    #     if self.event is not None:
    #         self.event.set()