
from hri_msgs.msg import TextToSpeechGoal as ISayGoal, TargetGoal as IGazeGoal, ExpressionGoal as IExpressionGoal, GestureGoal as IGestureGoal
from hri_api.util import ParamFormatting


class Default(object):
    pass


class SayGoal(ISayGoal):

    def __init__(self, sentence=""):
        ISayGoal.__init__(self, *args, **kwds)


class GazeGoal(IGazeGoal):

    def __init__(self, target, speed=0.5):
        IGazeGoal.__init__(self, )


class ExpressionGoal(IExpressionGoal):

    def __init__(self, expression, intensity=Default, speed=Default, duration=Default):
        IExpressionGoal.__init__(self)

        ParamFormatting.assert_types(self.__init__, expression, IExpression)
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

        ParamFormatting.assert_types(self.__init__, self.intensity, float)
        ParamFormatting.assert_range(self.__init__, self.intensity, 0.0, 1.0)

        ParamFormatting.assert_types(self.__init__, self.speed, float)
        ParamFormatting.assert_range(self.__init__, self.speed, 0.0, 1.0)

        ParamFormatting.assert_types(self.__init__, self.duration, float)
        ParamFormatting.assert_greater_than(self.__init__, self.duration, 0.0)