from math import sqrt
from shapely.geometry import Point


class vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Sensor:
    def __init__(self, *args):
        if (not isinstance(args[0], Point)):
            self.x = args[0]
            self.y = args[1]
            self.r = args[2]
            self.cover = []
        else:
            self.x = args[0].x
            self.y = args[0].y
            self.r = args[1]
            self.cover = args[2]

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, another):
        if not isinstance(another, type(self)):
            return NotImplemented
        return self.x == another.x and self.y == another.y

    def find_cover(self, targets):
        for t in targets:
            if t.distance(self) <= self.r + 0.0001:
                self.cover.append(t)


class Relay:
    def __init__(self, *args):
        if len(args) == 2:
            self.x = args[0]
            self.y = args[1]
        else:
            self.x = args[0].x
            self.y = args[0].y


class Target:
    def __init__(self, x, y, q, r):
        self.x = x
        self.y = y
        self.q = q
        self.r = r
        self.o_q = q

    def distance(self, another):
        return sqrt((self.x - another.x) ** 2 + (self.y - another.y) ** 2)

    def intersect(self, another):
        if abs(self.distance(another) - 2 * self.r) < 0.0000001:
            return [Sensor((self.x + another.x) / 2, (self.y + another.y) / 2, self.r)]
        if self.distance(another) < 2 * self.r:
            m = Point((self.x + another.x) / 2, (self.y + another.y) / 2)
            d = sqrt(self.r ** 2 - ((self.x - another.x) ** 2 + (self.y - another.y) ** 2) / 4)
            n1 = d / sqrt((self.x - another.x) ** 2 + (self.y - another.y) ** 2)
            n2 = -n1
            i1 = Sensor((m.x + (self.y - another.y) * n1), (m.y + (another.x - self.x) * n1), self.r)
            i2 = Sensor((m.x + (self.y - another.y) * n2), (m.y + (another.x - self.x) * n2), self.r)
            return [i1, i2]