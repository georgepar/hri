# Copyright (c) 2011 Robert Smallshire

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

'''
Predicate functions for testing Python entities to determine their type.
'''
from .portability import is_string as is_string_portable

__author__ = "Robert Smallshire"

def is_iterable(obj):
    '''Determine if an object is iterable.

    Args:
        obj: The object to be tested for supporting iteration.

    Returns:
        True if the object is iterable, otherwise False.
    '''
    try:
        iter(obj)
        return True
    except TypeError:
        return False

def is_type(obj):
    '''Determine if an object is a type.

    Args:
        obj: The object to be tested for being a type, or a tuple of types.

    Returns:
        True if the object is a type or tuple of types, otherwise False.
    '''
    try:
        isinstance(None, obj)
        return True
    except TypeError:
        return False

# TODO: Put is_callable in here too.

# TODO: is_string
def is_string(obj):
    return is_string_portable(obj)

# TODO: is_integer
