
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


class TestOfType(unittest.TestCase):

    def test_of_type_int(self):
        a = ['one', 2, 3, 'four', 'five', 6, 'seven', 8, 9, 'ten']
        b = Query(a).of_type(int).execute()
        c = [2, 3, 6, 8, 9]
        self.assertEqual(b, c)

    def test_of_type_str(self):
        a = ['one', 2, 3, 'four', 'five', 6, 'seven', 8, 9, 'ten']
        d = Query(a).of_type(str).execute()
        e = ['one', 'four', 'five', 'seven', 'ten']
        self.assertEqual(d, e)

    def test_of_type_tuple(self):
        a = ['one', 2, 3, 'four', 3.2, 'five', 6, 'seven', 8, 9, 'ten', 9.4]
        d = Query(a).of_type((str, int)).execute()
        e = ['one', 2, 3, 'four', 'five', 6, 'seven', 8, 9, 'ten']
        self.assertEqual(d, e)

    def test_of_type_not_type(self):
        a = ['one', 2, 3, 'four', 'five', 6, 'seven', 8, 9, 'ten']
        self.assertRaises(TypeError, lambda: Query(a).execute(7))
