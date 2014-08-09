#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# Copyright (c) 2014, James Diprose
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

# Author: Stuart Glaser

import threading
import time
import rospy
from rospy import Header
from actionlib_msgs.msg import *
from action_client import ActionClient, CommState, get_name_of_constant


class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
SimpleGoalState.to_string = classmethod(get_name_of_constant)


class ActionHandle():
    def __init__(self, goal_id, goal_handle, simple_state, done_cb, active_cb, feedback_cb):
        self.goal_id = goal_id
        self.goal_handle = goal_handle
        self.simple_state = simple_state
        self.done_cb = done_cb
        self.active_cb = active_cb
        self.feedback_cb = feedback_cb
        self.done_condition = threading.Condition()


class MultiGoalActionClient:
    ## @brief Constructs a SimpleActionClient and opens connections to an ActionServer.
    ##
    ## @param ns The namespace in which to access the action.  For
    ## example, the "goal" topic should occur under ns/goal
    ##
    ## @param ActionSpec The *Action message type.  The SimpleActionClient
    ## will grab the other message types from this type.
    def __init__(self, ns, ActionSpec):
        self.action_client = ActionClient(ns, ActionSpec)
        self.goal_callback = None
        self.preempt_callback = None

        self.action_handles_lock = threading.RLock()
        self.action_handles = {}

    ## @brief Blocks until the action server connects to this client
    ##
    ## @param timeout Max time to block before returning. A zero
    ## timeout is interpreted as an infinite timeout.
    ##
    ## @return True if the server connected in the allocated time. False on timeout
    def wait_for_server(self, timeout = rospy.Duration()):
        return self.action_client.wait_for_server(timeout)

    ## @brief Sends a goal to the ActionServer, and also registers callbacks
    ##
    ## If a previous goal is already active when this is called. We simply forget
    ## about that goal and start tracking the new goal. No cancel requests are made.
    ##
    ## @param done_cb Callback that gets called on transitions to
    ## Done.  The callback should take two parameters: the terminal
    ## state (as an integer from actionlib_msgs/GoalStatus) and the
    ## result.
    ##
    ## @param active_cb   No-parameter callback that gets called on transitions to Active.
    ##
    ## @param feedback_cb Callback that gets called whenever feedback
    ## for this goal is received.  Takes one parameter: the feedback.

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        goal_handle = self.action_client.send_goal(goal, self._handle_transition, self._handle_feedback)
        goal_id = self.get_goal_id(goal_handle)
        action_handle = ActionHandle(goal_id, goal_handle, SimpleGoalState.PENDING, done_cb, active_cb, feedback_cb)
        self.add_action_handle(action_handle)
        return goal_handle

    ## @brief Sends a goal to the ActionServer, waits for the goal to complete, and preempts goal is necessary
    ##
    ## If a previous goal is already active when this is called. We simply forget
    ## about that goal and start tracking the new goal. No cancel requests are made.
    ##
    ## If the goal does not complete within the execute_timeout, the goal gets preempted
    ##
    ## If preemption of the goal does not complete withing the preempt_timeout, this
    ## method simply returns
    ##
    ## @param execute_timeout The time to wait for the goal to complete
    ##
    ## @param preempt_timeout The time to wait for preemption to complete
    ##
    ## @return The goal's state.
    def send_goal_and_wait(self, goal, execute_timeout=rospy.Duration(), preempt_timeout=rospy.Duration()):
        self.send_goal(goal)
        if not self.wait_for_result(execute_timeout):
            # preempt action
            rospy.logdebug("Canceling goal")
            self.cancel_goal()
            if self.wait_for_result(preempt_timeout):
                rospy.logdebug("Preempt finished within specified preempt_timeout [%.2f]", preempt_timeout.to_sec());
            else:
                rospy.logdebug("Preempt didn't finish specified preempt_timeout [%.2f]", preempt_timeout.to_sec());
        return self.get_state()

    ## @brief Blocks until this goal transitions to done
    ## @param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
    ## @return True if the goal finished. False if the goal didn't finish within the allocated timeout
    def wait_for_result(self, goal_handle, timeout=rospy.Duration()):
        with self.action_handles_lock:
            if self.is_tracking_goal(goal_handle):
                rospy.logerr("Called wait_for_goal_to_finish when no goal exists")
                return False

        action_handle = self.get_action_handle(goal_handle)

        timeout_time = rospy.get_rostime() + timeout
        loop_period = rospy.Duration(0.1)
        with action_handle.done_condition:
            while not rospy.is_shutdown():
                time_left = timeout_time - rospy.get_rostime()
                if timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                    break

                if action_handle.simple_state == SimpleGoalState.DONE:
                    break

                if time_left > loop_period or timeout == rospy.Duration():
                    time_left = loop_period

                action_handle.done_condition.wait(time_left.to_sec())

        return action_handle.simple_state == SimpleGoalState.DONE


    ## @brief Gets the Result of the current goal
    def get_result(self, goal_handle):
        return goal_handle.get_result()

    ## @brief Get the state information for this goal
    ##
    ## Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED,
    ## PREEMPTED, ABORTED, SUCCEEDED, LOST.
    ##
    ## @return The goal's state. Returns LOST if this
    ## SimpleActionClient isn't tracking a goal.
    def get_state(self, goal_handle):
        with self.action_handles_lock:
            if self.is_tracking_goal(goal_handle):
                rospy.logerr("Called get_state when no goal is running")
                return GoalStatus.LOST
            status = goal_handle.get_goal_status()

            if status == GoalStatus.RECALLING:
                status = GoalStatus.PENDING
            elif status == GoalStatus.PREEMPTING:
                status = GoalStatus.ACTIVE

            return status

    ## @brief Returns the current status text of the goal.
    ##
    ## The text is sent by the action server. It is designed to
    ## help debugging issues on the server side.
    ##
    ## @return The current status text of the goal.
    def get_goal_status_text(self, goal_handle):
        with self.action_handles_lock:
            return goal_handle.get_goal_status_text()

    ## @brief Cancels all goals currently running on the action server
    ##
    ## This preempts all goals running on the action server at the point that
    ## this message is serviced by the ActionServer.
    def cancel_all_goals(self):
        with self.action_handles_lock:
            self.action_client.cancel_all_goals()

    ## @brief Cancels all goals prior to a given timestamp
    ##
    ## This preempts all goals running on the action server for which the 
    ## time stamp is earlier than the specified time stamp
    ## this message is serviced by the ActionServer.

    def cancel_goals_at_and_before_time(self, time):
        with self.action_handles_lock:
            self.action_client.cancel_goals_at_and_before_time(time)

    ## @brief Cancels the goal that we are currently pursuing
    def cancel_goal(self, goal_handle):
        with self.action_handles_lock:
            if self.is_tracking_goal(goal_handle):
                goal_id = self.get_goal_id(goal_handle)
                goal_handle.cancel()
                self.action_handles.pop(goal_id)

    def get_goal_id(self, goal_handle):
        return goal_handle.comm_state_machine.action_goal.goal_id

    def add_action_handle(self, action_handle):
        with self.action_handles_lock:
            self.action_handles[action_handle.goal_id] = action_handle

    def get_action_handle(self, goal_handle):
        with self.action_handles_lock:
            return self.action_handles[self.get_goal_id(goal_handle)]

    def is_tracking_goal(self, goal_handle):
        with self.action_handles_lock:
            action_handle = self.get_action_handle(goal_handle)
            return action_handle.goal_id in self.action_handles

    def _handle_transition(self, goal_handle):
        comm_state = goal_handle.get_comm_state()

        error_msg = "Received comm state %s when in simple state %s with SimpleActionClient in NS %s" % \
            (CommState.to_string(comm_state), SimpleGoalState.to_string(self.simple_state), rospy.resolve_name(self.action_client.ns))

        with self.action_handles_lock:
            action_handle = self.get_action_handle(goal_handle)

            if comm_state == CommState.ACTIVE:
                if action_handle.simple_state == SimpleGoalState.PENDING:
                    action_handle.simple_state = SimpleGoalState.ACTIVE
                    if action_handle.active_cb:
                        action_handle.active_cb()
                elif action_handle.simple_state == SimpleGoalState.DONE:
                    rospy.logerr(error_msg)
            elif comm_state == CommState.RECALLING:
                if action_handle.simple_state != SimpleGoalState.PENDING:
                    rospy.logerr(error_msg)
            elif comm_state == CommState.PREEMPTING:
                if action_handle.simple_state == SimpleGoalState.PENDING:
                    action_handle.simple_state = SimpleGoalState.ACTIVE
                    if action_handle.active_cb:
                        action_handle.active_cb()
                elif action_handle.simple_state == SimpleGoalState.DONE:
                    rospy.logerr(error_msg)
            elif comm_state == CommState.DONE:
                if action_handle.simple_state in [SimpleGoalState.PENDING, SimpleGoalState.ACTIVE]:
                    action_handle.simple_state = SimpleGoalState.DONE
                    if action_handle.done_cb:
                        action_handle.done_cb(goal_handle.get_goal_status(), goal_handle.get_result())
                    with action_handle.done_condition:
                        action_handle.done_condition.notifyAll()
                elif action_handle.simple_state == SimpleGoalState.DONE:
                    rospy.logerr("SimpleActionClient received DONE twice")

    def _handle_feedback(self, goal_handle, feedback):
        with self.action_handles_lock:
            if self.is_tracking_goal(goal_handle):
                rospy.logerr("Got a feedback callback on a goal handle that we're not tracking. %s vs %s" % \
                 (goal_handle.comm_state_machine.action_goal.goal_id.id,
                  goal_handle.comm_state_machine.action_goal.goal_id.id))
                return

            action_handle = self.get_action_handle(goal_handle)

            if action_handle.feedback_cb:
                action_handle.feedback_cb(feedback)
