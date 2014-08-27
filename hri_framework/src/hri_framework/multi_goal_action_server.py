#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# Copyright (c) 2014, Jamie Diprose
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
#
# Author: Alexander Sorokin, Jamie Diprose
# Based on C++ simple_action_server.h by Eitan Marder-Eppstein


import roslib
roslib.load_manifest('hri_framework')
import rospy
from actionlib import ActionServer, SimpleActionServer, GoalID, ServerGoalHandle
from actionlib import GoalStatus
from actionlib.simple_action_server import SimpleActionServer
from hri_msgs.msg import GoalList
import threading


class MultiGoalActionServer():

    def __init__(self, name, ActionSpec, auto_start=True):
        self.preempt_request = False
        self.new_goal_preempt_request = False

        self.goal_callback = None
        self.preempt_callback = None

        self.goal_handle_lock = threading.RLock()
        self.goal_handles = {}
        self.preempt_requests = {}

        self.active_goals_pub = rospy.Publisher(name + "/active_goals", GoalList)
        self.action_server = ActionServer(name, ActionSpec, self.internal_goal_callback, self.internal_preempt_callback, auto_start);

    def start(self):
        self.action_server.start()

    ## @brief Allows users to register a callback to be invoked when a new goal is available
    ## @param cb The callback to be invoked
    def register_goal_callback(self, goal_callback):
        self.goal_callback = goal_callback

    ## @brief Allows users to register a callback to be invoked when a new preempt request is available
    ## @param cb The callback to be invoked
    def register_preempt_callback(self, preempt_callback):
        self.preempt_callback = preempt_callback

    ## @brief Allows  polling implementations to query about preempt requests
    ## @return True if a preempt is requested, false otherwise
    def is_preempt_requested(self, goal_handle):
        goal_id = goal_handle.get_goal_id().id
        return goal_id in self.preempt_requests

    ## @brief Allows  polling implementations to query about the status of the current goal
    ## @return True if a goal is active, false otherwise
    def is_active(self, goal_handle):
        goal_id = goal_handle.get_goal_id().id

        if goal_id not in self.goal_handles:
            return False

        status = goal_handle.get_goal_status().status
        return status == GoalStatus.ACTIVE or status == GoalStatus.PREEMPTING

    def get_goal_handle(self, goal_handle):
        with self.goal_handle_lock:
            goal_id = goal_handle.get_goal_id().id

            if goal_id in self.goal_handles:
                return self.goal_handles[goal_id]
            return None

    def add_goal_handle(self, goal_handle):
        with self.goal_handle_lock:
            self.goal_handles[goal_handle.get_goal_id().id] = goal_handle

    def remove_goal_handle(self, goal_handle):
        with self.goal_handle_lock:
            goal_id = goal_handle.get_goal_id().id

            if goal_id in self.goal_handles:
                self.goal_handles.pop(goal_id)

                if goal_id in self.preempt_requests:
                    self.preempt_requests.pop(goal_id)

    def publish_active_goals(self):
        with self.goal_handle_lock:
            active_goals = GoalList()
            print(str(self.goal_handles))

            for key, goal_handle in self.goal_handles.iteritems():
                active_goals.goal_list.append(goal_handle.get_goal_id())
            self.active_goals_pub.publish(active_goals)

    ## @brief Callback for when the MultiGoalActionServer receives a new goal and passes it on
    def internal_goal_callback(self, goal_handle):
        rospy.loginfo("Goal received: %s", str(goal_handle.get_goal_id().id))

        with self.goal_handle_lock:
            if self.goal_callback:
                self.set_accepted(goal_handle)
                self.goal_callback(goal_handle)
            else:
                raise Exception('goal_callback not registered')

    ## @brief Sets the status of the active goal to preempted
    ## @param  result An optional result to send back to any clients of the goal
    def set_preempted(self, goal_handle, result=None, text=""):
        if not result:
            result = self.get_default_result()
        with self.goal_handle_lock:
            rospy.logdebug("Setting the current goal as canceled");
            goal_handle.set_canceled(result, text)
            self.remove_goal_handle(goal_handle)

    ## @brief Callback for when the ActionServer receives a new preempt and passes it on
    def internal_preempt_callback(self, goal_handle):
        rospy.loginfo("Preempt received: %s", str(id(goal_handle)))

        with self.goal_handle_lock:
            goal_id = goal_handle.get_goal_id().id
            self.preempt_requests[goal_id] = True

            if self.preempt_callback:
                self.preempt_callback(goal_handle)

    ## @brief Accepts a new goal when one is available
    def set_accepted(self, goal_handle):
        with self.goal_handle_lock:
            rospy.loginfo("Accepting a new goal: %s", goal_handle)
            self.add_goal_handle(goal_handle)
            goal_handle.set_accepted("This goal has been accepted by the multi goal action server")
            self.publish_active_goals()

    ## @brief Sets the status of the active goal to succeeded
    ## @param  result An optional result to send back to any clients of the goal
    def set_succeeded(self, goal_handle, result=None, text=""):
        with self.goal_handle_lock:
            if not result:
                result = self.get_default_result()
            goal_handle.set_succeeded(result, text)
            self.remove_goal_handle(goal_handle)
            self.publish_active_goals()

    ## @brief Sets the status of the active goal to aborted
    ## @param  result An optional result to send back to any clients of the goal
    def set_aborted(self, goal_handle, result=None, text=""):
        with self.goal_handle_lock:
            if not result:
                result = self.get_default_result()
            goal_handle.set_aborted(result, text)
            self.remove_goal_handle(goal_handle)
            self.publish_active_goals()

    ## @brief Publishes feedback for a given goal
    ## @param  feedback Shared pointer to the feedback to publish
    def publish_feedback(self, goal_handle, feedback):
        with self.goal_handle_lock:
            goal_handle.publish_feedback(feedback)

    def get_default_result(self):
        return self.action_server.ActionResultType();


