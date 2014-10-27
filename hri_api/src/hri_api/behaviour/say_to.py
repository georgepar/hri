import abc



class SayToActionHandle(IActionHandle):

    def __init__(self, say_to_plan):
        IActionHandle.__init__(self)
        self.say_to_plan = say_to_plan

    def cancel_action(self):
        self.say_to_plan.cancel()

    def wait_for_result(self):
        self.say_to_plan.wait()




class IBehaviour(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, robot):
        self.robot = robot

    @abc.abstractmethod
    def start(self):
        """
        Start the behaviour. This method should be non blocking.
        """
        return

    @abc.abstractmethod
    def stop(self):
        """
        Stop the behaviour
        """
        return


class SayTo(IBehaviour):

    valid_words_regex = "\w+[']{0,1}\w*[!?,.]{0,1}"
    punctuation = "[' ']"
    spaces = "[,.]"

    def __init__(self, robot):
        IBehaviour.__init__(self, robot)

    def start(self):


    def stop(self):


    def __init__(self, robot):
        self.robot = robot
        self.audience = None
        self.current_gazee = None
        self.sentence = ''
        self.gesture_lookup = {}
        self.expression_lookup = {}
        self.gaze_change_locations = {}

        self.say_ah = None
        self.gaze_ah = None
        self.other_ahs = []
        self.preempt_requested = False
        self.lock = threading.RLock()
        self.thread = None

    def reset(self):
        self.audience = None
        self.current_gazee = None
        self.sentence = ''
        self.gesture_lookup = {}
        self.expression_lookup = {}
        self.gaze_change_locations = {}

        self.say_ah = None
        self.gaze_ah = None
        self.other_ahs = []
        self.preempt_requested = False
        self.lock = threading.RLock()
        self.thread = None

    def wait(self):
        self.thread.join()
        self.robot.wait(self.say_ah)
        self.robot.wait(*self.other_ahs)

    def cancel(self, robot):
        with self.lock:
            self.preempt_requested = True

            if self.say_ah:
                robot.cancel(self.gaze_ah)

            if self.gaze_ah:
                robot.cancel(self.gaze_ah)

            self.robot.cancel(*self.other_ahs)

    def execute(self):
        self.thread = Thread(target=self.__execute)
        self.thread.start()
        return SayToActionHandle(self)

    def __execute(self):
        audience = self.audience

        # Pick a person to gaze at initially
        if isinstance(audience, Entity):
            person = audience

            with self.lock:
                if not self.preempt_requested:
                    self.gaze_ah = self.robot.gaze(person.head)
        else:
            if isinstance(audience, list):
                audience = Query(audience)

            results = audience.sort_increasing(lambda p: p.distance_to(self.robot)).execute()

            if len(results) > 0:
                person = results[0]
                self.current_gazee = person

                with self.lock:
                    if not self.preempt_requested:
                        self.gaze_ah = self.robot.gaze(person.head)

        if self.gaze_ah is not None:
            self.robot.wait(self.gaze_ah)

        with self.lock:
            if not self.preempt_requested:
                self.say_ah = self.robot.say(self.sentence)

    def add_action_handle(self, action_handle):
        self.other_ahs.append(action_handle)

    @staticmethod
    def get_gaze_change_locations(text):
        sentences = re.split("[,.?!]", text)
        gaze_change_locs = {}

        length = 0
        for sentence in sentences:
            length += SayToPlan.num_words(sentence)
            gaze_change_locs[length + 1] = ''

        return gaze_change_locs

    @staticmethod
    def num_words(text):
        return len(re.findall(SayToPlan.valid_words_regex, text))

    @staticmethod
    def get_sentence(text):
        tree = ET.fromstring("<sayto>" + text + "</sayto>")
        text = ""

        for node in tree.iter():
            if node.tag == "sayto":
                if node.text is not None:
                    text += node.text + " "
            else:
                if node.text is not None:
                    text += node.text + " "

                if node.tail is not None:
                    text += node.text + " "

        return text.strip()

    @staticmethod
    def enum_contains(enum, name):
        for e in enum:
            if e.name == name:
                return True

        return False

    def parse_parameters(self, text, audience, expression_enum, gesture_enum, tts_duration_srv):
        self.reset()
        self.audience = audience

        ParamFormatting.assert_types(self.parse_parameters, text, str)
        ParamFormatting.assert_types(self.parse_parameters, audience, Entity, Query)

        # if not is_callable(tts_duration_srv):
        #     raise TypeError("parse_parameters() parameter tts_duration_srv={0} is not callable".format(tts_duration_srv))

        self.sentence = SayToPlan.get_sentence(text)  # Get sentence
        self.gaze_change_locations = SayToPlan.get_gaze_change_locations(self.sentence)

        # Get expressions and gestures
        xml_tree = ET.fromstring("<sayto>" + text + "</sayto>")
        seen_text = ''

        for node in xml_tree.iter():
            if node.tag == "sayto":
                if node.text is not None:
                    seen_text += node.text + " "
            else:
                start_word_i = SayToPlan.num_words(seen_text)

                if node.text is not None:
                    seen_text += node.text + " "

                end_word_i = SayToPlan.num_words(seen_text)

                goal_name = node.tag

                if SayToPlan.enum_contains(expression_enum, goal_name):
                    goal = ExpressionGoal()
                    goal.expression = goal_name
                    goal.intensity = 0.5
                    goal.speed = 0.5

                    if 'intensity' in node.attrib:
                        goal.intensity = float(node.attrib["intensity"])

                    if 'speed' in node.attrib:
                        goal.speed = float(node.attrib["speed"])

                    goal.duration = tts_duration_srv(self.sentence, start_word_i, end_word_i).duration
                    self.expression_lookup[start_word_i] = goal

                elif SayToPlan.enum_contains(gesture_enum, goal_name):
                    goal = GestureGoal()
                    goal.gesture = goal_name

                    if 'target' in node.attrib:     # Check if target is Entity
                        goal.target = node.attrib["target"]
                    else:
                        raise AttributeError('Please specify a target attribute for {0} gesture'.format(goal_name))

                    goal.duration = tts_duration_srv(self.sentence, start_word_i, end_word_i).duration
                    self.gesture_lookup[start_word_i] = goal

                else:
                    raise TypeError('No gesture or expression called: {0}'.format(goal_name))

                if node.tail is not None:
                    seen_text += node.tail + " "


    # Speaking, gazing and gesturing simultaneously
    def say_to(self, text, audience):
        if not self.tts_duration_found:
            self.wait_for_services(self.tts_duration_srv)
            self.tts_duration_found = True

        ParamFormatting.assert_types(self.say_to, text, str)
        ParamFormatting.assert_types(self.say_to, audience, Entity, Query, list)

        self.say_to_plan.parse_parameters(text, audience, self.expression_enum, self.gesture_enum, self.tts_duration_srv)

        ah = self.say_to_plan.execute()
        self.add_action_handle(ah)
        return ah

    def say_to_and_wait(self, text, audience):
        ah = self.say_to(text, audience)
        self.wait(ah)


    def __say_feedback(self, feedback):
        if feedback.current_word_index in self.say_to_plan.gaze_change_locations:
            if isinstance(self.say_to_plan.audience, Entity):
                person = self.say_to_plan.audience
                self.say_to_plan.current_gazee = person
                self.say_to_plan.gaze_ah = self.gaze(person.head)

            elif isinstance(self.say_to_plan.audience, Query):
                people = self.say_to_plan.audience.execute()

                if len(people) > 1:
                    #if self.say_to_plan.current_gazee in people:
                    #    people.remove(self.say_to_plan.current_gazee)

                    person = random.choice(people)
                    self.say_to_plan.current_gazee = person
                    self.say_to_plan.gaze_ah = self.gaze(person.head)
                elif len(people) == 1:
                    person = people[0]
                    self.say_to_plan.current_gazee = person
                    self.say_to_plan.gaze_ah = self.gaze(person.head)

        if feedback.current_word_index in self.say_to_plan.expression_lookup:
            expression = self.say_to_plan.expression_lookup[feedback.current_word_num]
            ahs = self.do(expression)
            self.say_to_plan.add_action_handle(ahs[0])

        if feedback.current_word_index in self.say_to_plan.gesture_lookup:
            gesture = self.say_to_plan.gesture_lookup[feedback.current_word_index]
            ahs = self.do(gesture)
            self.say_to_plan.add_action_handle(ahs[0])