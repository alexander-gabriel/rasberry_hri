#!/usr/bin/env python
import itertools
import json
import os
import rospy
import rosparam
from hashlib import sha256

from common.parameters import CONFIG_DIRECTORY, STATE_DIRECTORY, \
    LOG_DIRECTORY, ACTIVE_DIRECTORY, NS



class Config():

    def __init__(self, data):
        self.robot_id = data["robot"]
        self.parameters = data["parameters"]
        self.termination_time = data["termination_time"]
        self.experiment_label = "{}".format(data["experiment_label"])




        # self.launch_files = ["/home/rasberry/catkin_ws/src/rasberry_hri/launch/picker_mover.launch"]
        # self.launch_files = ["/home/rasberry/catkin_ws/src/rasberry_hri/launch/hri_agent.launch"]
        # self.robot_pose = PoseWithCovarianceStamped()
        # self.robot_pose.pose.pose.position.x = 11.649
        # self.robot_pose.pose.pose.position.y = 4.62
        # self.robot_pose.pose.pose.position.z = 0

    # def reset(self):
    #     self.behaviour_times = copy(self._behaviour_times)
    #     self.behaviours = copy(self._behaviours)

    # def get_bag_paths(self):
    #     bags = {}
    #     for _, behaviour in self.behaviours.items():
    #         if behaviour["type"] == "rosbag":
    #             bag_path = behaviour["filename"]
    #             bags[bag_path] = rosbag.Bag(bag_path)
    #     return bags

    # def update_behaviour_times(self):
    #     self.behaviour_times = sorted(self.behaviours.keys())
    #     self._behaviour_times = copy(self.behaviour_times)
    #     self._behaviours = copy(self.behaviours)

    # def add_parameter(self, label, values):
    #     if not isinstance(values, list):
    #         values = [values]
    #     self.parameters[label] = values

    # def get_next_behaviour_time(self):
    #     try:
    #         return self.behaviour_times[0]
    #     except IndexError:
    #         return float('inf')

    # def get_next_behaviour(self):
    #     return self.behaviours[self.behaviour_times.pop(0)]


    def _generate_parameter_set(self):
        try:
            return self.parameter_set
        except AttributeError:
            try:
                keys, values = zip(*sorted(self.parameters.items()))
                self.parameter_set = [dict(zip(keys, v))
                                      for v in itertools.product(*values)]
                for parameters in self.parameter_set:
                    parameters["experiment_id"] = sha256(
                        json.dumps(parameters, sort_keys=True)
                    ).hexdigest()
            except ValueError:
                self.parameter_set = [{}]
            return self.parameter_set

    def __iter__(self):
        self._generate_parameter_set()
        self.max = len(self.parameter_set)
        return self

    def next(self):
        return self.__next__()

    def __next__(self):
        if self.state["config_index"] < self.max:
            x = self.parameter_set[self.state["config_index"]]
            with open(os.path.join(CONFIG_DIRECTORY,
                                   STATE_DIRECTORY,
                                   self.experiment_label + ".json"),
                      "w") as file:
                json.dump(self.state, file, sort_keys=True, indent=4,
                          separators=(',', ': '))
            self.state["config_index"] += 1
            return x
        else:
            raise StopIteration
