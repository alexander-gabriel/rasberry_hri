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
        'Neck-X': pi,
        'Neck-Y': pi,
        'Neck-Z': pi,
        'Right:Wrist-X': pi,
        'Right:Wrist-Y': pi,
        'Left:Wrist-X': pi,
        'Left:Wrist-Y': pi,
        'Right:Elbow-X': pi,
        'Right:Elbow-Y': pi,
        'Left:Elbow-X': pi,
        'Left:Elbow-Y': pi,
        'Right:Shoulder-X': pi,
        'Right:Shoulder-Y': pi,
        'Right:Shoulder-Z': pi,
        'Left:Shoulder-X': pi,
        'Left:Shoulder-Y': pi,
        'Left:Shoulder-Z': pi,
        'Upper-Spine-X': pi,
        'Upper-Spine-Y': pi,
        'Upper-Spine-Z': pi,
        'Mid-Spine-X': pi,
        'Mid-Spine-Y': pi,
        'Mid-Spine-Z': pi,
        'Lower-Spine-X': pi,
        'Lower-Spine-Y': pi,
        'Lower-Spine-Z': pi,
        'Right:Hip-X': pi,
        'Right:Hip-Y': pi,
        'Right:Hip-Z': pi,
        'Mid:Hip-X': pi,
        'Mid:Hip-Y': pi,
        'Mid:Hip-Z': pi,
        'Left:Hip-X': pi,
        'Left:Hip-Y': pi,
        'Left:Hip-Z': pi,
        'Right:Knee-X': pi,
        'Left:Knee-X': pi,
        'Right:Ankle-X': pi,
        'Right:Ankle-Y': pi,
        'Right:Ankle-Z': pi,
        'Left:Ankle-X': pi,
        'Left:Ankle-Y': pi,
        'Left:Ankle-Z': pi
        }.copy()


def get_position_prototype():
    return {
        'Neck-X': pi,
        'Neck-Y': pi,
        'Right:Wrist-X': pi,
        'Right:Wrist-Y': pi,
        'Left:Wrist-X': pi,
        'Left:Wrist-Y': pi,
        'Right:Elbow-X': pi,
        'Right:Elbow-Y': pi,
        'Left:Elbow-X': pi,
        'Left:Elbow-Y': pi,
        'Right:Shoulder-X': pi,
        'Right:Shoulder-Y': pi,
        'Left:Shoulder-X': pi,
        'Left:Shoulder-Y': pi,
        'Upper-Spine-X': pi,
        'Upper-Spine-Y': pi,
        'Mid-Spine-X': pi,
        'Mid-Spine-Y': pi,
        'Lower-Spine-X': pi,
        'Lower-Spine-Y': pi,
        'Right:Hip-X': pi,
        'Right:Hip-Y': pi,
        'Left:Hip-X': pi,
        'Left:Hip-Y': pi,
        'Right:Knee-X': pi,
        'Right:Knee-Y': pi,
        'Left:Knee-X': pi,
        'Left:Knee-Y': pi,
        'Right:Ankle-X': pi,
        'Right:Ankle-Y': pi,
        'Left:Ankle-X': pi,
        'Left:Ankle-Y': pi}.copy()


def get_position_prototype_3d():
    return {
        'Neck-X': pi,
        'Neck-Y': pi,
        'Neck-Z': pi,
        'Right:Wrist-X': pi,
        'Right:Wrist-Y': pi,
        'Right:Wrist-Z': pi,
        'Left:Wrist-X': pi,
        'Left:Wrist-Y': pi,
        'Left:Wrist-Z': pi,
        'Right:Elbow-X': pi,
        'Right:Elbow-Y': pi,
        'Right:Elbow-Z': pi,
        'Left:Elbow-X': pi,
        'Left:Elbow-Y': pi,
        'Left:Elbow-Z': pi,
        'Right:Shoulder-X': pi,
        'Right:Shoulder-Y': pi,
        'Right:Shoulder-Z': pi,
        'Left:Shoulder-X': pi,
        'Left:Shoulder-Y': pi,
        'Left:Shoulder-Z': pi,
        'Upper-Spine-X': pi,
        'Upper-Spine-Y': pi,
        'Upper-Spine-Z': pi,
        'Mid-Spine-X': pi,
        'Mid-Spine-Y': pi,
        'Mid-Spine-Z': pi,
        'Lower-Spine-X': pi,
        'Lower-Spine-Y': pi,
        'Lower-Spine-Z': pi,
        'Right:Hip-X': pi,
        'Right:Hip-Y': pi,
        'Right:Hip-Z': pi,
        'Left:Hip-X': pi,
        'Left:Hip-Y': pi,
        'Left:Hip-Z': pi,
        'Right:Knee-X': pi,
        'Right:Knee-Y': pi,
        'Right:Knee-Z': pi,
        'Left:Knee-X': pi,
        'Left:Knee-Y': pi,
        'Left:Knee-X': pi,
        'Right:Ankle-X': pi,
        'Right:Ankle-Y': pi,
        'Right:Ankle-Z': pi,
        'Left:Ankle-X': pi,
        'Left:Ankle-Y': pi,
        'Left:Ankle-Z': pi}.copy()


def mean(l):
    return sum(l)/len(l)
