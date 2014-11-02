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

# Author: Stuart Glaser, Jamie Diprose

import threading
import time
import rospy
from rospy import Header
from actionlib_msgs.msg import *
from actionlib.action_client import CommState, get_name_of_constant
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


class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
SimpleGoalState.to_string = classmethod(get_name_of_constant)


class ActionHandle():
    def __init__(self, goal_handle, simple_state, done_cb, active_cb, feedback_cb):
        self.goal_handle = goal_handle
        self.simple_state = simple_state
        self.done_cb = done_cb
        self.active_cb = active_cb
        self.feedback_cb = feedback_cb
        self.done_condition = threading.Condition()
        self.cleanup_start_time = None
        self.duration_inactive = None

    def goal_id(self):
        return self.goal_handle.comm_state_machine.action_goal.goal_id.id


class MultiGoalActionClient:
    CLEANUP_RATE_HZ = 1.0 / 60.0
    ## @brief Constructs a SimpleActionClient and opens connections to an ActionServer.
    ##
    ## @param ns The namespace in which to access the action.  For
    ## example, the "goal" topic should occur under ns/goal
    ##
    ## @param ActionSpec The *Action message type.  The SimpleActionClient
    ## will grab the other message types from this type.
    def __init__(self, ns, ActionSpec, queue_size=100):
        self.action_client = ActionClient(ns, ActionSpec)
        self.goal_callback = None
        self.preempt_callback = None
        self.queue_size = queue_size

        self.action_handles_lock = threading.RLock()
        self.action_handles = {}

        self.running = True
        self.cleanup_rate = rospy.Rate(MultiGoalActionClient.CLEANUP_RATE_HZ)
        self.cleanup_thread = threading.Thread(target=self.cleanup)
        self.cleanup_thread.start()

    def stop(self):
        self.running = False

    # Cleans up the stored inactive actions handles when they exceed the queue size
    def cleanup(self):
        while not rospy.is_shutdown() or self.running:
            with self.action_handles_lock:
                finished_actions = []
                for key, action_handle in self.action_handles.iteritems():
                    state = self.get_state(action_handle.goal_handle)

                    if state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
                        if action_handle.cleanup_start_time is None:
                            action_handle.cleanup_start_time = rospy.Time().now()
                            action_handle.duration_inactive = rospy.Duration()
                        else:
                            action_handle.duration_inactive = rospy.Time().now() - action_handle.cleanup_start_time

                        finished_actions.append(action_handle)

                finished_actions.sort(key=lambda action_handle: action_handle.duration_inactive.secs)
                to_delete = finished_actions[self.queue_size-1:]

                for action_handle in to_delete:
                    self.action_handles.pop(action_handle)

            self.cleanup_rate.sleep()

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
        action_handle = ActionHandle(goal_handle, SimpleGoalState.PENDING, done_cb, active_cb, feedback_cb)
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
        goal_handle = self.send_goal(goal)
        action_handle = ActionHandle(goal_handle, SimpleGoalState.PENDING, done_cb=None, active_cb=None, feedback_cb=None)
        self.add_action_handle(action_handle)

        if not self.wait_for_result(goal_handle, execute_timeout):
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
        if not self.is_tracking_goal(goal_handle):
            rospy.logerr("Called wait_for_result when no goal exists")
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
        with self.action_handles_lock:
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
            if not self.is_tracking_goal(goal_handle):
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
                goal_handle.cancel()

    def add_action_handle(self, action_handle):
        with self.action_handles_lock:
            self.action_handles[action_handle.goal_id()] = action_handle

    def get_action_handle(self, goal_handle):
        with self.action_handles_lock:
            if self.is_tracking_goal(goal_handle):
                goal_id = self.goal_id(goal_handle)
                return self.action_handles[goal_id]
            else:
                rospy.logerr('Error, not tracking goal')

    def goal_id(self, goal_handle):
        with self.action_handles_lock:
            return goal_handle.comm_state_machine.action_goal.goal_id.id

    def is_tracking_goal(self, goal_handle):
        with self.action_handles_lock:
            goal_id = self.goal_id(goal_handle)
            return goal_id in self.action_handles

    def _handle_transition(self, goal_handle):
        with self.action_handles_lock:
            comm_state = goal_handle.get_comm_state()

            action_handle = self.get_action_handle(goal_handle)

            error_msg = "Received comm state %s when in simple state %s with SimpleActionClient in NS %s" % \
            (CommState.to_string(comm_state), SimpleGoalState.to_string(action_handle.simple_state), rospy.resolve_name(self.action_client.ns))

            if comm_state == CommState.ACTIVE:
                if action_handle.simple_state == SimpleGoalState.PENDING:
                    action_handle.simple_state = SimpleGoalState.ACTIVE
                    if action_handle.active_cb:
                        action_handle.active_cb(goal_handle)
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
                        action_handle.done_cb(goal_handle)
                    with action_handle.done_condition:
                        action_handle.done_condition.notifyAll()
                elif action_handle.simple_state == SimpleGoalState.DONE:
                    rospy.logerr("SimpleActionClient received DONE twice")

    def _handle_feedback(self, goal_handle, feedback):
        with self.action_handles_lock:
            if not self.is_tracking_goal(goal_handle):
                rospy.logerr("Got a feedback callback on a goal handle that we're not tracking. %s vs %s" % \
                 (goal_handle.comm_state_machine.action_goal.goal_id.id,
                  goal_handle.comm_state_machine.action_goal.goal_id.id))
                return

            action_handle = self.get_action_handle(goal_handle)

            if action_handle.feedback_cb:
                action_handle.feedback_cb(goal_handle, feedback)
