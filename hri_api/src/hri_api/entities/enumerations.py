from enum import Enum


class Speed(Enum):
    slowly = 1
    moderately = 2
    quickly = 3

    def normalised(self):
        if self is Speed.slowly:
            value = 0.1
        elif self is Speed.moderately:
            value = 0.4
        elif self is Speed.quickly:
            value = 0.6

        return value


class Intensity(Enum):
    nothing = 1
    weakly = 2
    mildly = 3
    strongly = 4

    def normalised(self):
        if self is Intensity.nothing:
            value = 0.1
        elif self is Intensity.weakly:
            value = 0.4
        elif self is Intensity.mildly:
            value = 0.8
        elif self is Intensity.strongly:
            value = 1.0

        return value


class Gesture(Enum):
    pass


class Expression(Enum):
    pass
