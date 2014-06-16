

class Saliency(Entity):
    def __init__(self, entity_id):
        Entity.__init__(self, 'saliency', entity_id=entity_id)
        World().add_create_entity_callback(Saliency.create_person)

    @staticmethod
    def create_saliency(entity_msg):
        return Saliency(entity_msg.number)

    def say_to_gaze_tf_id(self):
        return self.head.base_link()

    def say_to_gesture_tf_id(self):
        return self.base_link()



