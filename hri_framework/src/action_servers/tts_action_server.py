#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
import rospy
import actionlib
import abc
from hri_framework.srv import TextToSpeechSubsentenceDuration, TextToSpeechSubsentenceDurationResponse
from hri_framework.msg import TextToSpeechFeedback, TextToSpeechResult, TextToSpeechAction
from robot_params import RobotParams

class TextToSpeechActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.robot_params = None
        self.server = None
        self.feedback = None
        self.result = None

    def start_server(self):
        rospy.init_node("text_to_speech_action_server", anonymous=True)
        self.robot_params = RobotParams()

        rospy.Service(self.robot_params.robot_id() + '_tts_subsentence_duration', TextToSpeechSubsentenceDuration, self.__tts_subsentence_duration)
        self.server = actionlib.SimpleActionServer(self.robot_params.robot_id() + '_text_to_speech', TextToSpeechAction, auto_start=False)
        self.server.register_goal_callback(self.__process_goal)
        self.server.register_preempt_callback(self.__cancel_goal)

        self.feedback = TextToSpeechFeedback()
        self.result = TextToSpeechResult()

    def __process_goal(self):
        #Check to see if goal currently running, if it is then cancel it.
        if self.server.is_active():
            self.__cancel_goal()

        #Accept new goal
        goal = self.server.accept_new_goal()

        #Execute goal
        self.synthesise_sentence(goal.sentence)

    def __cancel_goal(self):
        self.cancel_tts_synthesis()
        self.server.set_aborted(TextToSpeechResult(), "tts synthesis cancelled")
        rospy.loginfo("Goal cancelled.")

    def __tts_subsentence_duration(self, req):
        cdef str sentence = req.sentence
        cdef float duration = self.tts_subsentence_duration(sentence, req.start_word_index, req.end_word_index)
        return TextToSpeechSubsentenceDurationResponse(duration)

    def send_feedback(self, int current_word_index):
        """ Call this method when the current word being synthesised changes """

        self.feedback.current_word_index = current_word_index
        self.server.publish_feedback(self.feedback)
        rospy.loginfo("Feedback sent.")

    def synthesis_finished(self):
        """ Call this method when the entire sentence has been synthesised """
        self.server.set_succeeded(self.result)
        rospy.loginfo("Synthesis finished.")

    @abc.abstractmethod
    def cancel_tts_synthesis(self):
        """ Cancel tts synthesis if it is currently running. """
        return

    @abc.abstractmethod
    def synthesise_sentence(self, str sentence):
        """ Synthesise the sentence, e.g. using your robots text to speech synthesiser. """
        return

    @abc.abstractmethod
    def tts_subsentence_duration(self, str sentence, int start_word_index, int end_word_index):
        """ Return the duration (how long it takes to speak, double) a subset of a sentence that will be synthesised.
            The subset of the sentence is given via the parameters: start_word_index and end_word_index.
        """
        return