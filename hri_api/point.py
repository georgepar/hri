

cdef class Point():
    def __init__(self, double x, double y, double z):
        self.x = x
        self.y = y
        self.z = z

    def set_x(self, double x):
        self.x = x

    def set_y(self, double y):
        self.y = y

    def set_z(self, double z):
        self.z = z

    cdef Point add(self, Point b):
        return Point(self.x + b.x, self.y + b.y, self.z + b.z)

    cdef Point subtract(self, Point b):
        return Point(self.x - b.x, self.y - b.y, self.z - b.z)

    cdef Point multiply(self, double constant):
        return Point(self.x * constant, self.y * constant, self.z * constant)

    cdef Point normalize(self):
        length = self.length()
        return Point(self.x / length, self.y / length, self.z / length)

    cdef double length(self):
        return math.sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z))

    cdef double distance_to(self, Point other):
        return math.sqrt(math.pow(self.x - other.x, 2) + math.pow(self.y - other.y, 2) + math.pow(self.z - other.z, 2))

    def __repr__(self):
        return "x: " + str(self.x) + " y: " + str(self.y) + " z: " + str(self.z)