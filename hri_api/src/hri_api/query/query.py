# Copyright (c) 2014 Robert Smallshire

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
import heapq
import threading
import inspect
import rospy
from hri_api.math import Util
from .selectors import identity
from .types import (is_iterable, is_type)
from .portability import (is_callable, totally_ordered)
from hri_api.util import InitNode


class Query(object):

    def __init__(self, iterable, func=None):
        InitNode()

        if not is_iterable(iterable):
            raise TypeError("Cannot construct Query from non-iterable {0}".format(str(type(iterable))[7: -2]))

        if func is not None and not is_callable(func):
            raise TypeError('func must be callable')

        self.iterable = iterable
        self.func = func

    def __iter__(self):
        if self.func is not None:
            return iter(self.func())
        else:
            return iter(self.iterable)

    def get_id(self):
        return str(id(self))

    def select_type(self, classinfo):
        if not is_type(classinfo):
            raise TypeError("select_type() parameter classinfo={0} is not a class "
                "object or a type objector a tuple of class or "
                "type entities.".format(classinfo))

        return self.select_where(lambda x: isinstance(x, classinfo))

    def select_where(self, predicate):
        if not is_callable(predicate):
            raise TypeError("select_where() parameter predicate={predicate} is not "
                                  "callable".format(predicate=repr(predicate)))

        return Query(self, func=lambda: itertools.ifilter(predicate, self))

    def sort_increasing(self, key=identity):
        if not is_callable(key):
            raise TypeError("sort_increasing() parameter key_selector={key_selector} "
                    "is not callable".format(key_selector=repr(key)))

        return OrderedQuery(self, -1, key)

    def sort_decreasing(self, key=identity):
        if not is_callable(key):
            raise TypeError("sort_decreasing() parameter key_selector={0} "
                            "is not callable".format(repr(key)))

        return OrderedQuery(self, 1, key)

    def take(self, n):
        Util.assert_type(n, (int, long))
        n = max(0, n)
        return Query(self, func=lambda: itertools.islice(self, n))

    def execute(self):
        if isinstance(self.iterable, list):
            lst = self.iterable
            return lst
        lst = list(self)
        return lst


class OrderedQuery(Query):

    def __init__(self, iterable, order, func):
        '''Create an OrderedIterable.

            Args:
                iterable: The iterable sequence to be ordered.
                order: +1 for ascending, -1 for descending.
                func: The function to select the sorting key.
        '''

        if not is_iterable(iterable):
            raise TypeError("Cannot construct OrderedQuery from non-iterable {0}".format(str(type(iterable))[7: -2]))

        Util.assert_type(order, int)

        if not is_callable(func):
            raise TypeError('func is not callable')

        assert abs(order) == 1, 'order argument must be +1 or -1'
        super(OrderedQuery, self).__init__(iterable)
        self.funcs = [(order, func)]

    def then_increasing(self, key=identity):
        '''Introduce subsequent ordering to the sequence with an optional key.

        The returned sequence will be sorted in ascending order by the
        selected key.

        Note: This method uses deferred execution.

        Args:
            key_selector: A unary function the only positional argument to
                which is the element value from which the key will be
                selected.  The return value should be the key from that
                element.

        Returns:
            An OrderedQueryable over the sorted items.

        Raises:
            ValueError: If the OrderedQueryable is closed().
            TypeError: If key_selector is not callable.
        '''

        if not is_callable(key):
            raise TypeError("then_increasing() parameter key_selector={key_selector} "
                    "is not callable".format(key_selector=repr(key)))

        self.funcs.append((-1, key))
        return self

    def then_decreasing(self, key=identity):
        '''Introduce subsequent ordering to the sequence with an optional key.

        The returned sequence will be sorted in descending order by the
        selected key.

        Note: This method uses deferred execution.

        Args:
            key_selector: A unary function the only positional argument to
                which is the element value from which the key will be
                selected.  The return value should be the key from that
                element.

        Returns:
            An OrderedQueryable over the sorted items.

        Raises:
            ValueError: If the OrderedQueryable is closed().
            TypeError: If key_selector is not callable.
        '''

        if not is_callable(key):
            raise TypeError("then_decreasing() parameter key_selector={key_selector} is not callable".format(key_selector=repr(key)))

        self.funcs.append((+1, key))
        return self

    def __iter__(self):
        '''Support for the iterator protocol.

        Returns:
            An iterator object over the sorted elements.
        '''

        # Determine which sorting algorithms to use
        directions = [direction for direction, _ in self.funcs]
        direction_total = sum(directions)
        if direction_total == -len(self.funcs):
            # Uniform ascending sort - do nothing
            MultiKey = tuple

        elif direction_total == len(self.funcs):
            # Uniform descending sort - invert sense of operators
            @totally_ordered
            class MultiKey(object):
                def __init__(self, t):
                    self.t = tuple(t)

                def __lt__(lhs, rhs):
                    # Uniform descending sort - swap the comparison operators
                    return lhs.t > rhs.t

                def __eq__(lhs, rhs):
                    return lhs.t == rhs.t
        else:
            # Mixed ascending/descending sort - override all operators
            @totally_ordered
            class MultiKey(object):
                def __init__(self, t):
                    self.t = tuple(t)

                # TODO: [asq 1.1] We could use some runtime code generation here to compile a custom comparison operator
                def __lt__(lhs, rhs):
                    for direction, lhs_element, rhs_element in zip(directions, lhs.t, rhs.t):
                        cmp = (lhs_element > rhs_element) - (rhs_element > lhs_element)
                        if cmp == direction:
                            return True
                        if cmp == -direction:
                            return False
                    return False

                def __eq__(lhs, rhs):
                    return lhs.t == rhs.t

        # Uniform ascending sort - decorate, sort, undecorate using tuple element
        def create_key(index, item):
            return MultiKey(func(item) for _, func in self.funcs)

        lst = [(create_key(index, item), index, item) for index, item in enumerate(self.iterable)]
        heapq.heapify(lst)
        while lst:
            key, index, item = heapq.heappop(lst)
            yield item