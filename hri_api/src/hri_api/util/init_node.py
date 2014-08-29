import rospy
from hri_api.util import Singleton


class InitNode():
    __metaclass__ = Singleton

    def __init__(self):
        rospy.init_node("hri_application", anonymous=True)
