#!/usr/bin/env python
import rospy
from experiment import Experiment


class Experimenter:

    def __init__(self):
        rospy.loginfo("EXP: Initializing Experimenter")
        self.last_clock = 0
        self.configs = []
        self.current_config = None

    def run(self):
        for config in self.configs:
            self.current_config = config
            for parameters in config:
                experiment = Experiment(parameters, config)
                experiment.setup()
                experiment.spin()
                # config.reset()
        rospy.core.signal_shutdown('timeout')

    def add_config(self, config):
        self.configs.append(config)
