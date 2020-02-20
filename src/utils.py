import rospy
import os
from contextlib import contextmanager
from math import sqrt
from subprocess import Popen
from threading import Thread

from opencog.atomspace import AtomSpace, types, TruthValue
from opencog.type_constructors import *
from opencog.utilities import initialize_opencog


atomspace = AtomSpace()
initialize_opencog(atomspace)
set_type_ctor_atomspace(atomspace)

TRUE = TruthValue(1,1)
FALSE = TruthValue(0,1)

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


    def __len__(self):
        return self.items.__len__()


    def __getitem__(self, key):
        return self.items.__getitem__(key)


    def __delitem__(self, key):
        return self.items.__delitem__(key)



def run(cmd, stdout, stderr):
    """Run a given `cmd` in a subprocess, write logs to stdout / stderr.

    Parameters
    ----------
    cmd : list of str
      Command to run.
    stdout : str or subprocess.PIPE object
      Destination of stdout output.
    stderr : str or subprocess.PIPE object
      Destination of stderr output.

    Returns
    -------
    A subprocess.Popen instance.
    """
    return Popen(cmd, stdout=stdout, stderr=stderr, shell=False, preexec_fn=os.setsid)


def start_process(cmd, typ, start_time, dpath_logs):
    """Start a subprocess with the given command `cmd`.

    Parameters
    ----------
    cmd : list of str
      Command to run.
    typ : str
      Type of subprocess. This will be included in the logs' file names.
    start_time : str
      Datetime string, will be included in the logs' file names as well as
      the resulting bag's name.
    dpath_logs :
      Path to log direcotry.

    Returns
    -------
    A subprocess.Popen instance.
    """
    print('Starting', typ.upper())
    stdout, stderr = get_stdout_stderr(typ, start_time, dpath_logs)
    with open(stdout, 'wb') as out, open(stderr, 'wb') as err:
        return run(cmd, stdout=out, stderr=err)


def wp2sym(waypoint):
    return waypoint


def combine_terms(terms):
    return zip(terms)


def is_at(thing, place):
    link = StateLink(thing, place)
    return link


def is_a(thing, category):
    link = InheritanceLink(thing, category)
    return link


def not_same(thing1, thing2):
    link = NotLink(IdenticalLink(thing1, thing2))
    return link


def colocated(thing1, thing2): # ??
    link = EvaluationLink(PredicateNode("colocated"), ListLink(thing1, thing2))
    return link


def leads_to(origin, destination):
    link = EvaluationLink(
        PredicateNode("leads_to"),
        ListLink(origin, destination))
    return link


def free_path(origin, destination):
    link = EvaluationLink(
        PredicateNode("linked"),
        ListLink(origin, destination))
    return link


def called_robot(picker):
    link = StateLink(ListLink(picker, PredicateNode("called robot")), ConceptNode("TRUE"))
    return link


def not_called_robot(picker):
    link = StateLink(ListLink(picker, PredicateNode("called robot")), ConceptNode("FALSE"))
    return link


def approaching(picker):
    link = StateLink(ListLink(picker, PredicateNode("movement")), ConceptNode("APPROACHING"))
    return link


def leaving(picker):
    link = StateLink(ListLink(picker, PredicateNode("movement")), ConceptNode("LEAVING"))
    return link


def standing(picker):
    link = StateLink(ListLink(picker, PredicateNode("movement")), ConceptNode("STANDING"))
    return link


def seen_picking(picker):
    link = StateLink(ListLink(picker, PredicateNode("seen_picking")), ConceptNode("TRUE"))
    return link


def not_seen_picking(picker):
    link = StateLink(ListLink(picker, PredicateNode("seen_picking")), ConceptNode("FALSE"))
    return link


def sym2wp(symbol):
    return symbol


def get_angle_prototype():
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
        'Left:Ankle-Z': -1.0
        }.copy()


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
