
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
            return True
        else:
            return False


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
    return waypoint

def combine_terms(terms):
    return zip(terms)

def is_at(thing, place):
    t = VariableNode(thing)
    p = VariableNode(place)
    return ((t,p), StateLink(t, p))


def colocated(thing1, thing2): # ??
    t1 = VariableNode(thing1)
    t2 = VariableNode(thing2)
    return ((t1,t2), EvaluationLink(PredicateNode("colocated"), ListLink(t1, t2))))


def leads_to(origin, destination):
    o = VariableNode(origin)
    d = VariableNode(destination)
    return ((o,d), EvaluationLink(
        PredicateNode("leads_to"),
        ListLink(o,d)))


def has_crate(picker):
    p = VariableNode(picker)
    return ((p), StateLink(p, PredicateNode("has_crate")))


def seen_picking(picker):
    p = VariableNode(picker)
    return ((p), StateLink(p, PredicateNode("seen_picking")))


def sym2wp(symbol):
    return symbol


def get_angle_prototype():
    return {
        'Neck-RX': -1.0,
        'Neck-RY': -1.0,
        'Neck-RZ': -1.0,
        'Right:Wrist-RX': -1.0,
        'Right:Wrist-RY': -1.0,
        'Left:Wrist-RX': -1.0,
        'Left:Wrist-RY': -1.0,
        'Right:Elbow-RX': -1.0,
        'Right:Elbow-RY': -1.0,
        'Left:Elbow-RX': -1.0,
        'Left:Elbow-RY': -1.0,
        'Right:Shoulder-RX': -1.0,
        'Right:Shoulder-RY': -1.0,
        'Right:Shoulder-RZ': -1.0,
        'Left:Shoulder-RX': -1.0,
        'Left:Shoulder-RY': -1.0,
        'Left:Shoulder-RZ': -1.0,
        'Upper-Spine-RX': -1.0,
        'Upper-Spine-RY': -1.0,
        'Upper-Spine-RZ': -1.0,
        'Mid-Spine-RX': -1.0,
        'Mid-Spine-RY': -1.0,
        'Mid-Spine-RZ': -1.0,
        'Lower-Spine-RX': -1.0,
        'Lower-Spine-RY': -1.0,
        'Lower-Spine-RZ': -1.0,
        'Right:Hip-RX': -1.0,
        'Right:Hip-RY': -1.0,
        'Right:Hip-RZ': -1.0,
        'Left:Hip-RX': -1.0,
        'Left:Hip-RY': -1.0,
        'Left:Hip-RZ': -1.0,
        'Right:Knee-RX': -1.0,
        'Left:Knee-RX': -1.0,
        'Right:Ankle-RX': -1.0,
        'Right:Ankle-RY': -1.0,
        'Right:Ankle-RZ': -1.0,
        'Left:Ankle-RX': -1.0,
        'Left:Ankle-RY': -1.0,
        'Left:Ankle-RZ': -1.0}.copy()


def get_position_prototype():
    return {
        'Neck-X': None,
        'Neck-Y': None,
        'Right:Wrist-X': None,
        'Right:Wrist-Y': None,
        'Left:Wrist-X': None,
        'Left:Wrist-Y': None,
        'Right:Elbow-X': None,
        'Right:Elbow-Y': None,
        'Left:Elbow-X': None,
        'Left:Elbow-Y': None,
        'Right:Shoulder-X': None,
        'Right:Shoulder-Y': None,
        'Left:Shoulder-X': None,
        'Left:Shoulder-Y': None,
        'Upper-Spine-X': None,
        'Upper-Spine-Y': None,
        'Mid-Spine-X': None,
        'Mid-Spine-Y': None,
        'Lower-Spine-X': None,
        'Lower-Spine-Y': None,
        'Right:Hip-X': None,
        'Right:Hip-Y': None,
        'Left:Hip-X': None,
        'Left:Hip-Y': None,
        'Right:Knee-X': None,
        'Right:Knee-Y': None,
        'Left:Knee-X': None,
        'Left:Knee-Y': None,
        'Right:Ankle-X': None,
        'Right:Ankle-Y': None,
        'Left:Ankle-X': None,
        'Left:Ankle-Y': None}.copy()


def get_position_prototype_3d():
    return {
        'Neck-X': -1.0,
        'Neck-Y': -1.0,
        'Neck-Z': -1.0,
        'Right:Wrist-X': -1.0,
        'Right:Wrist-Y': -1.0,
        'Right:Wrist-Z': -1.0,
        'Left:Wrist-X': -1.0,
        'Left:Wrist-Y': -1.0,
        'Left:Wrist-Z': -1.0,
        'Right:Elbow-X': -1.0,
        'Right:Elbow-Y': -1.0,
        'Right:Elbow-Z': -1.0,
        'Left:Elbow-X': -1.0,
        'Left:Elbow-Y': -1.0,
        'Left:Elbow-Z': -1.0,
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
        'Right:Knee-Y': -1.0,
        'Right:Knee-Z': -1.0,
        'Left:Knee-X': -1.0,
        'Left:Knee-Y': -1.0,
        'Left:Knee-X': -1.0,
        'Right:Ankle-X': -1.0,
        'Right:Ankle-Y': -1.0,
        'Right:Ankle-Z': -1.0,
        'Left:Ankle-X': -1.0,
        'Left:Ankle-Y': -1.0,
        'Left:Ankle-Z': -1.0}.copy()



def mean(l):
    return sum(l)/len(l)
