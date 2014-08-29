#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')


class ActionHandle(object):
    def __init__(self, action_server, goal_id):
        self.namespace = self.action_server.action_client.ns
        self.action_server = action_server
        self.goal_id = goal_id

    def cancel(self):
        self.action_server.cancel_goal()

