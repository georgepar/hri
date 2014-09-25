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
from hri_msgs.msg import EntityMsg


#World().add_create_entity_callback(Person.create_person)
class Person(Entity):
    ENTITY_TYPE = 'person'

    def __init__(self, local_id):
        Entity.__init__(self, Person.ENTITY_TYPE, Person.ENTITY_TYPE + str(local_id), None)
        head = 'head'
        neck = 'neck'
        left_shoulder = 'left_shoulder'
        left_elbow = 'left_elbow'
        left_hand = 'left_hand'
        right_shoulder = 'right_shoulder'
        right_elbow = 'right_elbow'
        right_hand = 'right_hand'
        torso = 'torso'

        self.head = Head(head, head, self)
        self.neck = Neck(neck, neck, self)
        self.torso = Torso(torso, torso, self)
        self.left_hand = LeftHand(left_hand, left_hand, self)
        self.right_hand = RightHand(right_hand, right_hand, self)

    @classmethod
    def make(cls, local_id):
        return Person(local_id)

    def default_tf_frame_id(self):
        return self.head.tf_frame_id()

    def said_to(self, interlocutor, start_time, end_time):
        pass


class Head(Entity):
    def __init__(self, entity_type, local_id, parent):
        Entity.__init__(self, entity_type, local_id, parent)


class Neck(Entity):
    def __init__(self, entity_type, local_id, parent):
        Entity.__init__(self, entity_type, local_id, parent)


class Torso(Entity):
    def __init__(self, entity_type, local_id, parent):
        Entity.__init__(self, entity_type, local_id, parent)


class LeftHand(Entity):
    def __init__(self, entity_type, local_id, parent):
        Entity.__init__(self, entity_type, local_id, parent)


class RightHand(Entity):
    def __init__(self, entity_type, local_id, parent):
        Entity.__init__(self, entity_type, local_id, parent)
