from enum import Enum


class IGesture(Enum):

    def __init__(self, default_duration):
        self.default_duration = default_duration

    def __new__(cls, *args):
        value = len(cls.__members__) + 1
        obj = object.__new__(cls)
        obj._value_ = value
        return obj


class IExpression(Enum):

    def __init__(self, default_duration, default_speed, default_intensity):
        self.default_duration = default_duration
        self.default_speed = default_speed
        self.default_intensity = default_intensity

    def __new__(cls, *args):
        value = len(cls.__members__) + 1
        obj = object.__new__(cls)
        obj._value_ = value
        return obj
