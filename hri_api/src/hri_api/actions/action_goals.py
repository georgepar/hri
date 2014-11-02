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


class Default(object):
    pass


class SayGoal(ISayGoal):

    @requires(str, (None, Callable), (None, Callable))
    def __init__(self, sentence, feedback_cb=None, done_cb=None):
        ISayGoal.__init__(self)
        self.sentence = sentence
        self.feedback_cb = feedback_cb
        self.done_cb = done_cb


class GazeGoal(IGazeGoal):

    @requires(Entity, float, (None, Callable), (None, Callable))
    def __init__(self, head_target, speed=0.5, feedback_cb=None, done_cb=None):
        IGazeGoal.__init__(self)
        self.head_target = head_target.get_id()
        self.speed = speed
        self.feedback_cb = feedback_cb
        self.done_cb = done_cb

    def serialize(self, buff):
        if self.speed < 0.0 or self.speed > 1.0:
            rospy.logwarn("GazeGoal speed value of {0} is out of range, clipping to normal bounds: 0.0 <= speed <= 1.0".format(self.speed))
            self.speed = numpy.clip(self.speed, 0.0, 1.0)

        IGazeGoal.serialize(self, buff)


class GestureGoal(IGestureGoal):

    @requires(IGesture, (Default, float), (None, Callable), (None, Callable))
    def __init__(self, gesture, duration=Default(), feedback_cb=None, done_cb=None):
        IGestureGoal.__init__(self)
        self.gesture = gesture.name

        if duration is Default:
            self.duration = gesture.default_duration
        else:
            self.duration = duration

        self.feedback_cb = feedback_cb
        self.done_cb = done_cb

    def serialize(self, buff):
        if self.duration < 0.0:
            rospy.logwarn("GestureGoal duration value of {0} is out of range, clipping to normal bounds: duration > 0".format(self.duration))
            self.duration = max(0.0, self.duration)

        IGestureGoal.serialize(buff)


class ExpressionGoal(IExpressionGoal):

    @requires(IExpression, (Default, float), (Default, float), (Default, float), (None, Callable), (None, Callable))
    def __init__(self, expression, intensity=Default(), speed=Default(), duration=Default(), feedback_cb=None, done_cb=None):
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
