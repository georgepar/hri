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


__author__ = 'Robert Smallshire'


class Record(object):
    '''A class to which any attribute can be added at construction.'''

    def __init__(self, **kwargs):
        '''Initialise a Record with an attribute for each keyword argument.

        The attributes of a Record are mutable and may be read from and written
        to using regular Python instance attribute syntax.

        Args:
            **kwargs: Each keyword argument will be used to initialise an
                attribute with the same name as the argument and the given
                value.
        '''
        # TODO: Do we need to check for duplicates?
        self.__dict__.update(kwargs)

    def __eq__(self, rhs):
        return self.__dict__ == rhs.__dict__

    def __ne__(self, rhs):
        return self.__dict__ != rhs.__dict__

    def __str__(self):
        '''A string representation of the Record.'''
        return "Record(" + ', '.join(str(key) + '=' + str(value) for key, value in self.__dict__.items()) + ')'

    def __repr__(self):
        '''A valid Python expression string representation of the Record.'''
        return "Record(" + ', '.join(str(key) + '=' + repr(value) for key, value in self.__dict__.items()) + ')'


def new(**kwargs):
    '''A convenience factory for creating Records.

    Args:
        **kwargs: Each keyword argument will be used to initialise an
            attribute with the same name as the argument and the given
            value.

    Returns:
        A Record which has a named attribute for each of the keyword arguments.
    '''
    return Record(**kwargs)