#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_api')
import yaml


class RobotConfigParser(object):

    @staticmethod
    def load_robot_type(config_file_path):
        with open(config_file_path, 'r') as file:
            config = yaml.load(file)
            robot_type = config['robot']['type']
            return robot_type

    @staticmethod
    def load_gestures(config_file_path):
        with open(config_file_path, 'r') as file:
            config = yaml.load(file)
            gestures = config['robot']['gestures']
            return gestures

    @staticmethod
    def load_facial_expressions(config_file_path):
        with open(config_file_path, 'r') as file:
            config = yaml.load(file)
            config = yaml.load(file)
            facial_expressions = config['robot']['facial_expressions']
            return facial_expressions

