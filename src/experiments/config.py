#!/usr/bin/env python
import itertools
from copy import copy

import rosbag


class Config():

    def __init__(self, robot_id, experiment_id):
        self.robot_id = robot_id
        self.parameters = {}
        # self.behaviours = {}
        self.experiment_id = experiment_id
        # self.parameters = {"called_robot": [True],
        #                     "seen_picking": [False],
        #                     "picker_speed": [0.8],
        #                     "target_picker": ["Picker02"],
        #                     "iterations": ["90"]}
        # self.behaviours = {
        #                     1: {"type": "rosbag",
        #                         "label": "picking_berries",
        #                         "start": 0,
        #                         "duration": 3,
        #                         "filename": "/data/subject-1-out/picking_berries-34-joints.bag",
        #                         "topics": ["/human_actions"]
        #                         },
        #                     0: {"type": "rosbag",
        #                         "label": "call",
        #                         "start": 0,
        #                         "duration": 2,
        #                         "filename": "/data/subject-1-out/gesture_call-75-joints.bag",
        #                         "topics": ["/human_actions"]
        #                         },
        #                     0: {"type": "message",
        #                         "target": "picker_movement",
        #                         "message": "deliver"
        #                         }
        #                     }
        # self.behaviour_times = sorted(self.behaviours.keys())
        # self.termination_time = 20
        self.launch_files = [
         "/home/rasberry/catkin_ws/src/rasberry_hri/launch/hri_agent.launch",
         "/home/rasberry/catkin_ws/src/rasberry_hri/launch/picker_mover.launch"
         ]
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

    def add_parameter(self, label, values):
        if not isinstance(values, list):
            values = [values]
        self.parameters[label] = values

    # def get_next_behaviour_time(self):
    #     try:
    #         return self.behaviour_times[0]
    #     except IndexError:
    #         return float('inf')

    # def get_next_behaviour(self):
    #     return self.behaviours[self.behaviour_times.pop(0)]

    def get_parameter_set(self):
        try:
            return self.parameter_set
        except AttributeError:
            try:
                keys, values = zip(*self.parameters.items())
                self.parameter_set = [dict(zip(keys, v))
                                      for v in itertools.product(*values)]
            except ValueError:
                self.parameter_set = [{}]
            return self.parameter_set
