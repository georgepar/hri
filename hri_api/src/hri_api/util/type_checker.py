
class ArgumentTypeError(TypeError):
    """
    An argument is the wrong type.
    """
    def __init__(self, func_name, expected, actual):
        expected_formatted = []

        for i, item in enumerate(expected):
            if isinstance(item, tuple):
                type_names = str(tuple(map(lambda j: j.__name__, item))).replace(",", " or ").replace('(', "").replace(')', "")
                expected_formatted.append(type_names)
            else:
                expected_formatted.append(item.__name__)

        expected_formatted = str(tuple(expected_formatted)).replace("\"", "").replace(",)", ")")
        actual_formatted = str(tuple(map(lambda i: i.__name__, actual))).replace(",)", ")")

        self.error = "'{0}' method should return {1}, but was given {2}".format(func_name, expected_formatted, actual_formatted)

    def __str__(self):
        return self.error


# class ReturnTypeError(TypeError):
#     """
#     The return value is the wrong type.
#     """
#     def __init__(self, func_name, expected, actual):
#         expected_formatted = ''
#
#         if isinstance(expected, tuple):
#             expected_formatted = str(tuple(map(lambda j: j.__name__, expected))).replace(",", " or ").replace('(', "").replace(')', "")
#         else:
#             expected_formatted = "'{0}'".format(expected.__name__)
#
#         actual_formatted = "'{0}'".format(actual.__name__)
#
#         self.error = "'{0}' method accepts ({1}), but result is ({2})".format(func_name, expected_formatted, actual_formatted)
#
#     def __str__(self):
#         return self.error


class TypeChecker():

    @staticmethod
    def accepts(func_name, expected_types, *args):
        actual_types = tuple(map(type, args))

        for a_type, e_type in zip(actual_types, expected_types):
            if isinstance(e_type, tuple):
                if a_type not in e_type:
                    raise ArgumentTypeError(func_name, expected_types, actual_types)
            elif a_type != e_type:
                raise ArgumentTypeError(func_name, expected_types, actual_types)

# def returns(expected):
#
#     def decorator(orig_func):
#         def new_func(*args):
#             result = orig_func(*args)
#             actual = type(result)
#
#             if isinstance(expected, tuple):
#                 if actual not in expected:
#                     raise ReturnTypeError(orig_func.__name__, expected, actual)
#             elif actual != expected:
#                 raise ReturnTypeError(orig_func.__name__, expected, actual)
#             else:
#                 return result
#         new_func.__name__ = orig_func.__name__
#         return new_func
#     return decorator

