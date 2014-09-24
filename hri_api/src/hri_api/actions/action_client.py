#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from actionlib import ActionException, GoalManager,GoalStatusArray, GoalID, ActionClient as SingleGoalActionClient
import rospy


class ActionClient(SingleGoalActionClient):

    # Overriding the constructor because it only lets you publish one message to pub_goal / pub_cancel
    def __init__(self, ns, ActionSpec):
        self.ns = ns
        self.last_status_msg = None

        try:
            a = ActionSpec()

            self.ActionSpec = ActionSpec
            self.ActionGoal = type(a.action_goal)
            self.ActionResult = type(a.action_result)
            self.ActionFeedback = type(a.action_feedback)
        except AttributeError:
            raise ActionException("Type is not an action spec: %s" % str(ActionSpec))

        self.pub_goal = rospy.Publisher(rospy.remap_name(ns) + '/goal', self.ActionGoal, queue_size=100)
        self.pub_cancel = rospy.Publisher(rospy.remap_name(ns) + '/cancel', GoalID, queue_size=100)

        self.manager = GoalManager(ActionSpec)
        self.manager.register_send_goal_fn(self.pub_goal.publish)
        self.manager.register_cancel_fn(self.pub_cancel.publish)

        self.status_sub = rospy.Subscriber(rospy.remap_name(ns) + '/status', GoalStatusArray, self._status_cb)
        self.result_sub = rospy.Subscriber(rospy.remap_name(ns) + '/result', self.ActionResult, self._result_cb)
        self.feedback_sub = rospy.Subscriber(rospy.remap_name(ns) + '/feedback', self.ActionFeedback, self._feedback_cb)