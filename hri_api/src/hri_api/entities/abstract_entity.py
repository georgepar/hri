#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
import abc

class AbstractEntity(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def is_infront_of(self, other_entity):
        """ Return whether 'self' is in front of 'other_entity' """
        return

    @abc.abstractmethod
    def is_behind(self, other_entity):
        """ Return whether 'self' is behind 'other_entity' """
        return

    @abc.abstractmethod
    def is_left_of(self, other_entity):
        """ Return whether 'self' is left of 'other_entity' """
        return

    @abc.abstractmethod
    def is_right_of(self, other_entity):
        """ Return whether 'self' is right of 'other_entity' """
        return

    @abc.abstractmethod
    def distance_to(self, other_entity):
        """ Return the distance between 'self' and 'other_entity' """
        return

    @abc.abstractmethod
    def get_id(self):
        """
        :return: a unique numeric id for the entity
        """

