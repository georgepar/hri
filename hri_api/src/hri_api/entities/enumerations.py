from enum import Enum


class IGesture(Enum):

    def default_duration(self):
        raise NotImplementedError('please implement this method')

    def get_data(self):
        for i in self.data:
            if self is i[0]:
                return i


class IExpression(Enum):

    def default_duration(self):
        raise NotImplementedError('please implement this method')

    def default_speed(self):
        raise NotImplementedError('please implement this method')

    def default_intensity(self):
        raise NotImplementedError('please implement this method')

    def get_data(self):
        for i in self.data:
            if self is i[0]:
                return i