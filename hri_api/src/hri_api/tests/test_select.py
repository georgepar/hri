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

import unittest
from hri_api.query import Query
from hri_api.tests import infinite, TracingGenerator

__author__ = "Robert Smallshire"

class TestSelect(unittest.TestCase):

    def test_select(self):
        a = range(0, 100)
        b = Query(a).select(lambda x: x % 3 == 0).to_list()
        c = list(range(0, 100, 3))
        self.assertEqual(b, c)

    def test_select_not_callable(self):
        a = range(0, 100)
        self.assertRaises(TypeError, lambda: Query(a).select("not callable"))

    def test_select_infinite(self):
        a = infinite()
        b = Query(a).select(lambda x: x % 5 == 0).take(3).to_list()
        c = [0, 5, 10]
        self.assertEqual(b, c)

    def test_select_deferred(self):
        a = TracingGenerator()
        self.assertEqual(a.trace, [])
        b = Query(a).select(lambda x: x % 3 == 0)
        self.assertEqual(a.trace, [])
        c = b.take(2).to_list()
        self.assertEqual(a.trace, [0, 1, 2, 3])

