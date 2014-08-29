# Copyright (c) 2011-2014 Robert Smallshire, Jamie Diprose

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
test_queryable.py Unit tests for asq.queryable.Queryable
'''
import unittest

from hri_api.query import Query

def times_two(x):
    return 2 * x

def times(x, y):
    return x * y

def infinite():
    i = 0
    while True:
        yield i
        i += 1

class TracingGenerator(object):

    def __init__(self):
        self._trace = []
        self._i = 0

    def __iter__(self):
        while True:
            self._trace.append(self._i)
            yield self._i
            self._i += 1

    trace = property(lambda self: self._trace)

class TestQueryable(unittest.TestCase):

    def test_non_iterable(self):
        self.assertRaises(TypeError, lambda: Query(5))

def inc_chr(y):
    return chr(ord(y)+1)

def randgen():
    import random
    while True:
        yield random.random()

if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestQueryable)
    unittest.TextTestRunner(verbosity=2).run(suite)