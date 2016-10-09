__author__ = 'cogbot'
import rospy
import inspect
import re

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
    def assert_types(method, parameter, param_name, *types):
        method_name = method.__name__

        if not isinstance(parameter, types):
            msg = "{0}() parameter {1}={2} is not part of type {3}".format(method_name, param_name, parameter, types)
            rospy.logerr(msg)

        return parameter

    @staticmethod
    def ensure_range(method, parameter, param_name, minimum, maximum):
        if parameter < minimum or parameter > maximum:
            method_name = method.__name__

            if parameter < minimum:
                trimmed_val = minimum
            elif parameter > maximum:
                trimmed_val = maximum

            msg = "{0}() parameter {1}={2} is not between the range {3} - {4}. Trimming to {5}.".format(method_name, param_name, parameter, minimum, maximum, trimmed_val)
            rospy.logerr(msg)

            return trimmed_val
        return parameter

    @staticmethod
    def ensure_greater_than(method, parameter, param_name, minimum):
        if parameter <= minimum:
            method_name = method.__name__
            trimmed_val = minimum + abs(minimum * 0.01)
            msg = "{0}() parameter {1}={2} is not greater than {3}. Trimming to {4}.".format(method_name, param_name, parameter, minimum, trimmed_val)
            rospy.logerr(msg)
            return trimmed_val
        return parameter

    @staticmethod
    def ensure_greater_than_or_equal(method, parameter, param_name, minimum):
        if parameter < minimum:
            method_name = method.__name__
            msg = "{0}() parameter {1}={2} is not greater than or equal to {3}. Trimming to {4}.".format(method_name, param_name, parameter, minimum, minimum)
            rospy.logerr(msg)
            return minimum
        return parameter

    @staticmethod
    def ensure_less_than(method, parameter, param_name, maximum):
        if parameter >= maximum:
            method_name = method.__name__
            trimmed_val = maximum - abs(maximum * 0.01)
            msg = "{0}() parameter {1}={2} is not less than {3}. Trimming to {4}.".format(method_name, param_name, parameter, maximum, trimmed_val)
            rospy.logerr(msg)
            return trimmed_val
        return parameter

    @staticmethod
    def ensure_less_than_or_equal(method, parameter, param_name, maximum):
        if parameter > maximum:
            method_name = method.__name__
            msg = "{0}() parameter {1}={2} is not less than or equal to {3}. Trimming to {4}.".format(method_name, param_name, parameter, maximum, maximum)
            rospy.logerr(msg)
            return maximum
        return parameter
