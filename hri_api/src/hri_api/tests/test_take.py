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
from hri_api.tests import infinite

__author__ = "Robert Smallshire"

class TestTake(unittest.TestCase):

    def test_take_negative(self):
        a = ['a', 'b', 'c']
        b = Query(a).take(-1).to_list()
        c = []
        self.assertEqual(b, c)

    def test_take_zero(self):
        a = ['a', 'b', 'c']
        b = Query(a).take(0).to_list()
        c = []
        self.assertEqual(b, c)

    def test_take_five(self):
        a = ['a', 'b', 'c', 'd', 'e', 'f', 'g']
        b = Query(a).take(5).to_list()
        c = ['a', 'b', 'c', 'd', 'e']
        self.assertEqual(b, c)

    def test_take_too_many(self):
        a = ['a', 'b', 'c', 'd', 'e', 'f', 'g']
        b = Query(a).take(10).to_list()
        c = ['a', 'b', 'c', 'd', 'e', 'f', 'g']
        self.assertEqual(b, c)

    def test_take_from_infinite(self):
        b = Query(infinite()).take(5).to_list()
        c = [0, 1, 2, 3, 4]
        self.assertEqual(b, c)

