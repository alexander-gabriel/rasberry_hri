#!/usr/bin/env python
import itertools

import rosbag
from geometry_msgs.msg import PoseWithCovarianceStamped


        # self.behaviours = {120: {"start": None,
        #                     "duration": None,
        #                     "filename": None,
        #                     "label": None,
        #                     "type": "action"},
        #                 240: {"label": None,
        #                     "type": "movement"},
        #                     }

class Config():

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.parameters = {"called_robot": [False],
                            "target_picker": ["Picker02"]}
        self.behaviours = {
                            5: {"type": "message",
                                "target": "action_label",
                                "message": "picking berries left"},
                            8: {"type": "rosbag",
                                "label": "call",
                                "start": 76,
                                "duration": 1,
                                "filename": "/data/subject-1.bag",
                                "topics": ["/camera/color/image_raw"]
                                },
                            14: {"type": "message",
                                "target": "picker_movement",
                                "message": "exchange"}
                                }
        self.behaviour_times = sorted(self.behaviours.keys())
        self.termination_time = 60
        # self.launch_files = ["/home/rasberry/catkin_ws/src/rasberry_hri/launch/hri_agent.launch", "/home/rasberry/catkin_ws/src/rasberry_hri/launch/picker_mover.launch"]
        self.launch_files = ["/home/rasberry/catkin_ws/src/rasberry_hri/launch/picker_mover.launch"]
        self.robot_pose = PoseWithCovarianceStamped()
        self.robot_pose.pose.pose.position.x = 11.649
        self.robot_pose.pose.pose.position.y = 4.64
        self.robot_pose.pose.pose.position.z = 0.0

    def get_bag_paths(self):
        bags = {}
        for _, behaviour in self.behaviours.items():
            if behaviour["type"] == "rosbag":
                bag_path = behaviour["filename"]
                bags[bag_path] = rosbag.Bag(bag_path)
        return bags

    def add_parameter(self, label, values):
        if not isinstance(values, list):
            values = [values]
        self.parameters[label] = values


    def get_next_behaviour_time(self):
        try:
            return self.behaviour_times[0]
        except IndexError:
            return float('inf')


    def get_next_behaviour(self):
        return self.behaviours[self.behaviour_times.pop(0)]


    def get_parameter_set(self):
        try:
            return self.parameter_set
        except AttributeError:
            try:
                keys, values = zip(*self.parameters.items())
                self.parameter_set = [dict(zip(keys,v)) for v in itertools.product(*values)]
            except ValueError:
                self.parameter_set = [{}]
            return self.parameter_set
