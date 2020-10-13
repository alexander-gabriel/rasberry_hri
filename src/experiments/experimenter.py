#!/usr/bin/env python
import rospy
import roslaunch
from config import Config
from experiment import Experiment

# x TODO: storage of rosbag action data (start, end, filename, classification)
# x TODO: keeping track of time (subscribing to the simulation clock, launching events at predefined time steps)
# x TODO: start rosbag from experimenter
# x TODO: in worldstate: add trajectory calculus, generate labels for human movement relative to his node location as seen from the robot's perspective
# x TODO: generate human behavior events from observed movement
# x TODO: add distance metric and emergency stop
# x TODO: check reasoning
# x TODO: check rosbag player
# x TODO: check pose detection
# x TODO: check experimenter
# x TODO: annotate the latest rosbags
# x TODO: split the rosbags
# x TODO: add pose for picking berries
# x TODO: add pose for call
# x TODO: add pose for cancel
# x TODO: add pose for stop
# x TODO: add new detected actions to bdi system
# x FIX concurrency stopping message reception

# . TODO: add is_a(.., human) back in and fix inheritance reasoning
# . TODO: fix reasoning (not, is_a, linked)

# . TODO: make sure all important steps are logged (distance metric!!)

# ? TODO: in bdi system: pause, start simulation, change simulation realtime factor
# . TODO: implement log parser


class Experimenter:

    def __init__(self):
        rospy.loginfo("EXP: Initializing Experimenter")
        self.last_clock = 0
        self.configs = []
        self.current_config = None

    def run(self):
        for config in self.configs:
            self.current_config = config
            for parameters in config.get_parameter_set():
                experiment = Experiment(parameters, config)
                experiment.setup()
                experiment.spin()
                # config.reset()
        rospy.core.signal_shutdown('timeout')

    def add_config(self, config):
        self.configs.append(config)
