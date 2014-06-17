from unittest import TestCase
from hri_api.util import RobotConfigParser


class TestRobotConfigParser(TestCase):
    def test_load_robot_type(self):
        robot_type = RobotConfigParser.load_robot_type("/home/cogbot/catkin_ws/src/hri/hri_api/src/hri_api/tests/test_robot.yaml")
        self.assertEqual(robot_type, 'zeno')

    def test_load_gestures(self):
        gestures = RobotConfigParser.load_gestures("/home/cogbot/catkin_ws/src/hri/hri_api/src/hri_api/tests/test_robot.yaml")
        self.assertEqual(gestures, ['wave', 'point'])

    def test_load_facial_expressions(self):
        facial_expressions = RobotConfigParser.load_facial_expressions("/home/cogbot/catkin_ws/src/hri/hri_api/src/hri_api/tests/test_robot.yaml")
        self.assertEqual(facial_expressions, ['smile'])