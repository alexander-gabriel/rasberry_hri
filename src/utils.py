
from contextlib import contextmanager

@contextmanager
def suppress(*exceptions):
    try:
        yield
    except exceptions:
        pass

class OrderedConsistentSet:

    def __init__(self):
        self.items = []


    def append(self, item):
        if not item in self.items:
            try:
                if item.startswith("!"):
                    self.items.remove(item[1:])
                else:
                    self.items.remove("!" + item)
            except ValueError:
                pass
            self.items.append(item)


    def __iadd__(self, other):
        for item in other:
            self.append(item)
        return self


    def __contains__(self, key):
        return key in self.items


    def __str__(self):
        return self.items.__str__()


    def __iter__(self):
        return self.items.__iter__()

def wp2sym(waypoint):
    return waypoint.lower()

def sym2wp(symbol):
    return symbol.replace("waypoint", "WayPoint")


def get_model_prototype():
    return {
        'Neck-X': -1.0,
        'Neck-Y': -1.0,
        'Neck-Z': -1.0,
        'Right:Wrist-X': -1.0,
        'Right:Wrist-Y': -1.0,
        'Left:Wrist-X': -1.0,
        'Left:Wrist-Y': -1.0,
        'Right:Elbow-X': -1.0,
        'Right:Elbow-Y': -1.0,
        'Left:Elbow-X': -1.0,
        'Left:Elbow-Y': -1.0,
        'Right:Shoulder-X': -1.0,
        'Right:Shoulder-Y': -1.0,
        'Right:Shoulder-Z': -1.0,
        'Left:Shoulder-X': -1.0,
        'Left:Shoulder-Y': -1.0,
        'Left:Shoulder-Z': -1.0,
        'Upper-Spine-X': -1.0,
        'Upper-Spine-Y': -1.0,
        'Upper-Spine-Z': -1.0,
        'Mid-Spine-X': -1.0,
        'Mid-Spine-Y': -1.0,
        'Mid-Spine-Z': -1.0,
        'Lower-Spine-X': -1.0,
        'Lower-Spine-Y': -1.0,
        'Lower-Spine-Z': -1.0,
        'Right:Hip-X': -1.0,
        'Right:Hip-Y': -1.0,
        'Right:Hip-Z': -1.0,
        'Left:Hip-X': -1.0,
        'Left:Hip-Y': -1.0,
        'Left:Hip-Z': -1.0,
        'Right:Knee-X': -1.0,
        'Left:Knee-X': -1.0,
        'Right:Ankle-X': -1.0,
        'Right:Ankle-Y': -1.0,
        'Right:Ankle-Z': -1.0,
        'Left:Ankle-X': -1.0,
        'Left:Ankle-Y': -1.0,
        'Left:Ankle-Z': -1.0}.copy()


def mean(l):
    return sum(l)/len(l)
