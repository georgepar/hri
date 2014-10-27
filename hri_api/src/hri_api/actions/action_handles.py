class IActionHandle():
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        pass

    @abc.abstractmethod
    def cancel_action(self):
        """
        :return:
        """

    @abc.abstractmethod
    def wait_for_result(self):
        """

        :return:
        """


class SingleGoalActionHandle(IActionHandle):
    def __init__(self, action_client):
        IActionHandle.__init__(self)
        self.action_client = action_client

    def cancel_action(self):
        self.action_client.cancel_goal()

    def wait_for_result(self):
        self.action_client.wait_for_result()


class MultiGoalActionHandle(IActionHandle):
    def __init__(self, action_client, goal_handle):
        IActionHandle.__init__(self)
        self.action_client = action_client
        self.goal_handle = goal_handle

    def cancel_action(self):
        self.action_client.cancel_goal(self.goal_handle)

    def wait_for_result(self):
        self.action_client.wait_for_result(self.goal_handle)

