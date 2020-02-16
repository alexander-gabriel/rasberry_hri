#!/usr/bin/env python

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

# . FIX concurrency stopping message reception
# . TODO: make sure all important steps are logged (distance metric!!)
# . TODO: implement log parser
# . TODO: annotate the latest rosbags
# . TODO: split the rosbags 

# . TODO: add pose for picking berries
# . TODO: add pose for call
# . TODO: add pose for cancel
# . TODO: add pose for stop
# . TODO: in bdi system: pause, start simulation, change simulation realtime factor


class Experimenter:

    def __init__(self):
        print("EXP: Initializing Experimenter")
        self.last_clock = 0
        self.configs = []
        self.current_config = None
        print("EXP: Initialization finished")


    def run(self):
        for config in self.configs:
            self.current_config = config
            for parameters in config.get_parameter_set():
                print("EXP: Starting config")
                # this is one experiment run
                experiment = Experiment(parameters, config)
                experiment.setup()
                experiment.spin()


    def add_config(self, config):
        self.configs.append(config)
