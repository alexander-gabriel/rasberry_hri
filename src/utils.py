from threading import Lock
from contextlib import contextmanager
from math import sqrt, pi
import os
import sys

from subprocess import Popen
from threading import Thread

import rospy

from opencog.atomspace import AtomSpace, types, TruthValue
from opencog.type_constructors import *
from opencog.utilities import initialize_opencog





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


def sym2wp(symbol):
    return symbol


def get_angle_prototype():
    return {
        'Neck': pi,
        'Right:Wrist': pi,
        'Left:Wrist': pi,
        'Right:Elbow': pi,
        'Left:Elbow': pi,
        'Right:Shoulder': pi,
        'Left:Shoulder': pi,
        'Upper-Spine': pi,
        'Mid-Spine': pi,
        'Lower-Spine': pi,
        'Right:Hip': pi,
        'Mid:Hip': pi,
        'Left:Hip': pi,
        'Right:Knee': pi,
        'Left:Knee': pi,
        'Right:Ankle': pi,
        'Left:Ankle': pi,
        }.copy()


def get_position_prototype():
    return {
        'Neck': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Wrist': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Wrist': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Elbow': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Elbow': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Shoulder': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Shoulder': {'X': pi, 'Y': pi, 'P': 0},
        'Upper-Spine': {'X': pi, 'Y': pi, 'P': 0},
        'Mid-Spine': {'X': pi, 'Y': pi, 'P': 0},
        'Lower-Spine': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Hip': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Hip': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Knee': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Knee': {'X': pi, 'Y': pi, 'P': 0},
        'Right:Ankle': {'X': pi, 'Y': pi, 'P': 0},
        'Left:Ankle': {'X': pi, 'Y': pi, 'P': 0}}.copy()


def get_position_prototype_3d():
    return {
        'Neck': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Wrist': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Wrist': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Elbow': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Elbow': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Shoulder': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Shoulder': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Upper-Spine': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Mid-Spine': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Lower-Spine': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Hip': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Hip': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Knee': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Knee': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Right:Ankle': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0},
        'Left:Ankle': {'X': pi, 'Y': pi, 'Z': pi, 'P': 0}}.copy()


def mean(l):
    return sum(l)/len(l)
