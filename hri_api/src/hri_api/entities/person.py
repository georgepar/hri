# Copyright (c) 2014, James Diprose
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from hri_api.entities import Entity
from hri_api.util import ParamAssertions


class Person(Entity):
    """This class represents a person and their major body parts.

    .. note::
        You can instantiate the Person class directly: e.g. Person(1)
        or automatically with the perception system; see the tutorials
        for more information.

        The persons body parts are accessed via attributes, e.g. the
        persons head is represented by the attribute self.head

    """

    def __init__(self, local_id):
        """

        Instantiates a Person class and their body parts.

        :param local_id: the id that uniquely identifies this person instance from all other person instances
        :type local_id: int
        """

        # Check types
        ParamAssertions.assert_types(self.__init__, local_id, int)

        # Call the superclass constructor
        Entity.__init__(self, 'person' + str(local_id), None)

        # Create all of the persons body parts
        self.head = Head('head', self)
        self.neck = Neck('neck', self)
        self.torso = Torso('torso', self)

        self.left_shoulder = Shoulder('left_shoulder', self)
        self.left_elbow = Elbow('left_elbow', self)
        self.left_hand = Hand('left_hand', self)

        self.left_hip = Hip('left_hip', self)
        self.left_knee = Knee('left_knee', self)
        self.left_foot = Foot('left_foot', self)

        self.right_shoulder = Shoulder('right_shoulder', self)
        self.right_elbow = Elbow('right_elbow', self)
        self.right_hand = Hand('right_hand', self)

        self.right_hip = Hip('right_hip', self)
        self.right_knee = Knee('right_knee', self)
        self.right_foot = Foot('right_foot', self)

    @classmethod
    def make(cls, local_id):
        """

        The factory method for the Person class.

        :param local_id: the id that uniquely identifies this person instance from all other person instances
        :type local_id: int
        :return: the Person instance
        :rtype: Person
        """

        return Person(local_id)

    def default_body_part(self):
        """

        Gets the default body part for a Person instance. In this case it is the persons head.

        :return: the persons head
        :rtype: Head
        """

        return self.head


class Head(Entity):
    """A persons head.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)


class Neck(Entity):
    """A persons neck.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)


class Torso(Entity):
    """A persons torso.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)


class Shoulder(Entity):
    """A persons shoulder.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)


class Elbow(Entity):
    """A persons elbow.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)


class Hand(Entity):
    """A persons hand.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)


class Hip(Entity):
    """A persons hip.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)


class Knee(Entity):
    """A persons knee.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)


class Foot(Entity):
    """A persons foot.
    """

    def __init__(self, local_frame_id, parent):
        """

        Instantiates a persons head.

        :param local_frame_id: identifies this body parts coordinate within the transformation tree (tf) when it is concatenated with its parents local_frame_id
        :type local_frame_id: str
        :param parent: the parent Person instance
        :type parent: Person
        """

        Entity.__init__(self, local_frame_id, parent)