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

import itertools
import unittest
from hri_api.query import Query

__author__ = "Robert Smallshire"

def index_by_identity(sequence, obj):
    '''Returns the index of the instance in the sequence. Comparison is by
    object identity rather than object value. Raises ValueError if there is
    no such item.
    '''
    for index, item in enumerate(sequence):
        if item is obj:
            return index
    raise ValueError("index_by_identity(lst, x): x not in lst")

def sgn(i):
    if i < 0: return -1
    elif i > 0: return +1
    return 0

class TestOrderBy(unittest.TestCase):

    def test_order_by(self):
        a = [27, 74, 18, 48, 57, 97, 76, 20, 91, 8, 80, 59, 20, 32, 58, 12, 74, 78, 4]
        b = Query(a).order_by_ascending().to_list()
        c = [4, 8, 12, 18, 20, 20, 27, 32, 48, 57, 58, 59, 74, 74, 76, 78, 80, 91, 97]
        self.assertEqual(b, c)

    def test_order_by2(self):
        a = [1, 9, 7, 2, 5, 4, 6, 3, 8, 10]
        b = Query(a).order_by_ascending().to_list()
        c = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.assertEqual(b, c)

    def test_order_by_key(self):
        a = ['Sort', 'words', 'by', 'length']
        b = Query(a).order_by_ascending(len).to_list()
        c = ['by', 'Sort', 'words', 'length']
        self.assertEqual(b, c)

    def test_order_by_not_callable(self):
        a = ['Sort', 'words', 'by', 'length']
        self.assertRaises(TypeError, lambda: Query(a).order_by_ascending("not callable"))

    def test_order_by_stability(self):
        a = [1, 2, 3]
        b = [1, 2, 3]
        c = [4, 5, 6]
        d = [4, 5, 6]
        e = [7, 5, 4]
        f = [7, 5, 4]
        self.assertTrue(a is not b)
        self.assertTrue(c is not d)
        self.assertTrue(e is not f)
        s = [a, b, c, d, e, f]
        for pre_perm in itertools.permutations(s):
            pre_index_a = index_by_identity(pre_perm, a)
            pre_index_b = index_by_identity(pre_perm, b)
            pre_index_c = index_by_identity(pre_perm, c)
            pre_index_d = index_by_identity(pre_perm, d)
            pre_index_e = index_by_identity(pre_perm, e)
            pre_index_f = index_by_identity(pre_perm, f)
            pre_order_a_b = sgn(pre_index_a - pre_index_b)
            pre_order_c_d = sgn(pre_index_c - pre_index_d)
            pre_order_e_f = sgn(pre_index_e - pre_index_f)

            post_perm = Query(pre_perm).order_by_ascending().to_list()

            post_index_a = index_by_identity(post_perm, a)
            post_index_b = index_by_identity(post_perm, b)
            post_index_c = index_by_identity(post_perm, c)
            post_index_d = index_by_identity(post_perm, d)
            post_index_e = index_by_identity(post_perm, e)
            post_index_f = index_by_identity(post_perm, f)
            post_order_a_b = sgn(post_index_a - post_index_b)
            post_order_c_d = sgn(post_index_c - post_index_d)
            post_order_e_f = sgn(post_index_e - post_index_f)

            self.assertEqual(pre_order_a_b, post_order_a_b)
            self.assertEqual(pre_order_c_d, post_order_c_d)
            self.assertEqual(pre_order_e_f, post_order_e_f)


    def test_then_by(self):
        a = ['sort', 'these', 'words', 'by', 'length', 'and', 'then', 'lexicographically']
        b = Query(a).order_by_ascending(len).then_by().to_list()
        c = ['by', 'and', 'sort', 'then', 'these', 'words', 'length', 'lexicographically']
        self.assertEqual(b, c)

    def test_then_by_key(self):
        a = ['sort', 'using', 'third', 'letter', 'then', 'second']
        b = Query(a).order_by_ascending(lambda x: x[2]).then_by(lambda y: y[1]).to_list()
        c = ['second', 'then', 'third', 'using', 'sort', 'letter']
        self.assertEqual(b, c)

    def test_then_by_not_callable(self):
        a = ['sort', 'using', 'third', 'letter', 'then', 'second']
        b = Query(a).order_by_ascending(lambda x: x[2])
        self.assertRaises(TypeError, lambda: b.then_by("not callable"))

    def test_order_by_descending(self):
        a = [1, 9, 7, 2, 5, 4, 6, 3, 8, 10]
        b = Query(a).order_by_descending().to_list()
        c = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
        self.assertEqual(b, c)

    def test_order_by_descending_key(self):
        a = ['Sort', 'words', 'by', 'length']
        b = Query(a).order_by_descending(len).to_list()
        c = ['length', 'words', 'Sort', 'by']
        self.assertEqual(b, c)

    def test_order_by_descending_not_callable(self):
        a = ['Sort', 'words', 'by', 'length']
        self.assertRaises(TypeError, lambda: Query(a).order_by_descending("not callable"))

    def test_then_by_descending(self):
        a = ['sort', 'these', 'words', 'by', 'length', 'and', 'then', 'lexicographically']
        b = Query(a).order_by_ascending(len).then_by_descending().to_list()
        c = ['by', 'and', 'then', 'sort', 'words', 'these', 'length', 'lexicographically']
        self.assertEqual(b, c)

    def test_then_by_descending_key(self):
        a = ['sort', 'using', 'third', 'letter', 'then', 'second']
        b = Query(a).order_by_ascending(lambda x: x[2]).then_by_descending(lambda y: y[1]).to_list()
        c = ['second', 'then', 'using', 'third', 'sort', 'letter']
        self.assertEqual(b, c)

    def test_then_by_descending_not_callable(self):
        a = ['sort', 'using', 'third', 'letter', 'then', 'second']
        b = Query(a).order_by_ascending(lambda x: x[2])
        self.assertRaises(TypeError, lambda: b.then_by_descending("not callable"))
