#!/usr/bin/env python
import rospy
import actionlib
import abc
from hri_msgs.srv import TextToSpeechSubsentenceDuration, TextToSpeechSubsentenceDurationResponse
from hri_msgs.msg import TextToSpeechFeedback, TextToSpeechResult, TextToSpeechAction


class TextToSpeechActionServer():
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.server = None
        self.feedback = None
        self.result = None

    def start(self):
        rospy.Service('tts_subsentence_duration', TextToSpeechSubsentenceDuration, self.__tts_subsentence_duration)
        self.server = actionlib.SimpleActionServer('text_to_speech', TextToSpeechAction, auto_start=False)
        self.server.register_goal_callback(self.__process_goal)
        self.server.register_preempt_callback(self.__cancel_goal)

        self.feedback = TextToSpeechFeedback()
        self.result = TextToSpeechResult()
        self.server.start()

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
        sentence = req.sentence
        duration = self.tts_subsentence_duration(sentence, req.start_word_index, req.end_word_index)
        return TextToSpeechSubsentenceDurationResponse(duration)

    def send_feedback(self, current_word_index):
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
    def synthesise_sentence(self, sentence):
        """ Synthesise the sentence, e.g. using your robots text to speech synthesiser. """
        return

    @abc.abstractmethod
    def tts_subsentence_duration(self, sentence, start_word_index, end_word_index):
        """ Return the duration (how long it takes to speak, double) a subset of a sentence that will be synthesised.
            The subset of the sentence is given via the parameters: start_word_index and end_word_index.
        """
        return