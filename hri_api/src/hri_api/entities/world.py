
class World(object):
    def __init__(self):
        rospy.init_node("environment", anonymous=True)
        self.__objects = []
        self.__deleted_objects = []
        self.__obj_id_dict = {}
        self.__people_sub = rospy.Subscriber("openni_tracker/users", UInt16MultiArray, self.__people_changed)
        self.__get_tf_ids_service = rospy.Service('get_tf_ids_service', GetTfIds, self.__get_tf_ids)
        self.__object_change_lock = threading.Lock() #Needed if you are subscribing to entities from multiple sources

    cdef bool contains(self, Obj obj):
        if obj in self.__objects:
            return True
        return False

    cdef Obj get_obj(self, int obj_id):
        return self.__obj_id_dict[obj_id]

    def add_object(self, Obj obj, user_object = True):
        obj_id = id(obj)
        self.__obj_id_dict[obj_id] = obj

        if user_object:
            self.__user_objects.append(obj)

        self.__objects.append(obj)

        rospy.loginfo("Adding object: %s", id(obj))

    def remove_object(self, Obj obj):
        self.__objects.remove()
        self.__deleted_objects.append(obj)

    # Callbacks
    def __get_tf_ids(self, req):
        obj = self.__get_obj_from_obj_id(int(req.obj_id))
        tf_ids = obj.get_tf_ids()
        return GetTfIdsResponse(tf_ids)

    def __people_changed(self, msg):
        with self.__object_change_lock:
            peoples_ids = msg.data
            print "DATA: " + str(peoples_ids)
            print "adding people"

            # Add people
            for person_id in peoples_ids:
                print "object_id: " + str(person_id) + " type: " + str(type(person_id))
                person_exists = False
                objects = set(self.__objects) - set(self.__own_objects)
                for obj in objects:
                    if isinstance(obj, Person):
                        if obj.person_id == person_id:
                            person_exists = True
                            break

                if not person_exists:
                    # If object already exists in deleted entities
                    matches = [p for p in self.__deleted_objects if isinstance(p, Person) and p.person_id == person_id]

                    if len(matches) > 0:
                        person = matches[0]
                        self.__deleted_objects.remove(person)
                        self.__objects.append(person)
                    else:
                        person = Person(person_id)
                        self.add_object(person, own_object = False)

            ''' Delete people '''
            objects = set(self.__objects) - set(self.__own_objects)
            for obj in objects:
                person_exists = False
                for person_id in peoples_ids:
                    if isinstance(obj, Person):
                        if obj.person_id == person_id:
                            person_exists = True
                            break

                if not person_exists:
                    self.remove_object(obj)