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

from hri_msgs.msg import TextToSpeechGoal as ISayGoal, TargetGoal as IGazeGoal, ExpressionGoal as IExpressionGoal, GestureGoal as IGestureGoal
import numpy
import rospy
from hri_api.util import TypeChecker
import inspect
from collections import Callable
from hri_api.entities import Entity, IGesture, IExpression


class Default(object):
    pass


class SayGoal(ISayGoal):

    """
        Can be executed by passing it to Robots simultaneously or consecutively functions.
    """

    def __init__(self, text, feedback_cb=None, done_cb=None):

        """ The say action goal constructor.

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

        ISayGoal.__init__(self)
        self.text = text
        self.feedback_cb = feedback_cb
        self.done_cb = done_cb


class GazeGoal(IGazeGoal):

    """
        Can be executed by passing it to Robots simultaneously or consecutively functions.
    """

    def __init__(self, target, speed=0.5, feedback_cb=None, done_cb=None):

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

        IGazeGoal.__init__(self)
        self.head_target = target.get_id()
        self.speed = speed
        self.feedback_cb = feedback_cb
        self.done_cb = done_cb

    def serialize(self, buff):
        if self.speed < 0.0 or self.speed > 1.0:
            rospy.logwarn("GazeGoal speed value of {0} is out of range, clipping to normal bounds: 0.0 <= speed <= 1.0".format(self.speed))
            self.speed = numpy.clip(self.speed, 0.0, 1.0)

        IGazeGoal.serialize(self, buff)


class GestureGoal(IGestureGoal):

    """
        Can be executed by passing it to Robots simultaneously or consecutively functions.
    """

    def __init__(self, gesture, duration=Default(), target=None, feedback_cb=None, done_cb=None):

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

        IGestureGoal.__init__(self)

        self.gesture = gesture.name

        if duration is Default:
            self.duration = gesture.default_duration
        else:
            self.duration = duration

        self.target = target
        self.feedback_cb = feedback_cb
        self.done_cb = done_cb

    def serialize(self, buff):
        if self.duration < 0.0:
            rospy.logwarn("GestureGoal duration value of {0} is out of range, clipping to normal bounds: duration > 0".format(self.duration))
            self.duration = max(0.0, self.duration)

        IGestureGoal.serialize(buff)


class ExpressionGoal(IExpressionGoal):

    """
        Can be executed by passing it to Robots simultaneously or consecutively functions.
    """

    def __init__(self, expression, intensity=Default(), speed=Default(), duration=Default(), feedback_cb=None, done_cb=None):
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

        IExpressionGoal.__init__(self)
        self.expression = expression.name

        if intensity is Default:
            self.intensity = expression.default_intensity
        else:
            self.intensity = intensity

        if speed is Default:
            self.speed = expression.default_speed
        else:
            self.speed = speed

        if self.duration is Default:
            self.duration = expression.default_duration
        else:
            self.duration = duration

    def serialize(self, buff):
        if self.intensity < 0.0 or self.intensity > 1.0:
            rospy.logwarn("ExpressionGoal intensity value of {0} is out of range, clipping to normal bounds: 0.0 <= intensity <= 1.0".format(self.intensity))
            self.intensity = numpy.clip(self.intensity, 0.0, 1.0)

        if self.speed < 0.0 or self.speed > 1.0:
            rospy.logwarn("ExpressionGoal speed value of {0} is out of range, clipping to normal bounds: 0.0 <= intensity <= 1.0".format(self.speed))
            self.speed = numpy.clip(self.speed, 0.0, 1.0)

        if self.duration < 0.0:
            rospy.logwarn("ExpressionGoal duration value of {0} is out of range, clipping to normal bounds: duration > 0".format(self.duration))
            self.duration = max(0.0, self.duration)

        IExpressionGoal.serialize(buff)
