
class Util(object):
    @staticmethod
    def assert_type(instance, type):
        if not isinstance(instance, type):
            name = [ k for k,v in locals().iteritems() if v is instance][0]
            raise TypeError(name + ' must be a ' + type.__name__)
