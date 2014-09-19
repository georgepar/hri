__author__ = 'cogbot'

class GestureDoesNotExistError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

class FacialExpressionDoesNotExistError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class ParamFormatting():

    @staticmethod
    def instance_name(instance):
        instance_name = ''

        for k, v in list(locals().iteritems()):
            if v is instance:
                instance_name = k

        return instance_name

    @staticmethod
    def assert_types(method, parameter, *types):
        method_name = method.__name__
        param_name = ParamFormatting.instance_name(parameter)

        if not isinstance(parameter, types):
            raise TypeError("{0}() parameter {1}={2} is not part of type {3}".format(method_name, param_name, parameter, types))

    @staticmethod
    def assert_range(method, parameter, minimum, maximum):
        method_name = method.__name__
        param_name = ParamFormatting.instance_name(parameter)

        if not (minimum <= parameter <= maximum):
            raise TypeError("{0}() parameter {1}={2} is not between the range {3} - {4}".format(method_name, param_name, parameter, minimum, maximum))

    @staticmethod
    def assert_greater_than(method, parameter, other_value):
        method_name = method.__name__
        param_name = ParamFormatting.instance_name(parameter)

        if not (parameter > other_value):
            raise TypeError("{0}() parameter {1}={2} is not greater than {3}".format(method_name, param_name, parameter, other_value))

    @staticmethod
    def assert_greater_than_or_equal(method, parameter, other_value):
        method_name = method.__name__
        param_name = ParamFormatting.instance_name(parameter)

        if not (parameter >= other_value):
            raise TypeError("{0}() parameter {1}={2} is not greater than or equal to {3}".format(method_name, param_name, parameter, other_value))

    @staticmethod
    def assert_less_than(method, parameter, other_value):
        method_name = method.__name__
        param_name = ParamFormatting.instance_name(parameter)

        if not (parameter < other_value):
            raise TypeError("{0}() parameter {1}={2} is not less than {3}".format(method_name, param_name, parameter, other_value))

    @staticmethod
    def assert_less_than_or_equal(method, parameter, other_value):
        method_name = method.__name__
        param_name = ParamFormatting.instance_name(parameter)

        if not (value <= other_value):
            raise TypeError("{0}() parameter {1}={2} is not less than or equal to {3}".format(method_name, param_name, parameter, other_value))
