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

picking_filenames = [
        "subject-10-moving-out/picking_berries-23-joints.bag",
        "subject-10-out/picking_berries-18-joints.bag",
        "subject-1-moving-out/picking_berries-39-joints.bag",
        "subject-1-out/picking_berries-34-joints.bag",
        "subject-2-moving-out/picking_berries-10-joints.bag",
        "subject-2-out/picking_berries-14-joints.bag",
        "subject-3-moving-out/picking_berries-24-joints.bag",
        "subject-3-out/picking_berries-16-joints.bag",
        "subject-4-moving-out/picking_berries-25-joints.bag",
        "subject-4-out/picking_berries-20-joints.bag",
        "subject-5-moving-out/picking_berries-23-joints.bag",
        "subject-5-out/picking_berries-15-joints.bag",
        "subject-6-moving-out/picking_berries-17-joints.bag",
        "subject-6-out/picking_berries-17-joints.bag",
        "subject-7-moving-out/picking_berries-49-joints.bag",
        "subject-7-out/picking_berries-17-joints.bag",
        "subject-8-moving-out/picking_berries-24-joints.bag",
        "subject-8-out/picking_berries-16-joints.bag",
        "subject-9-moving-out/picking_berries-32-joints.bag",
        "subject-9-out/picking_berries-22-joints.bag",
]



call_filenames = [
        "subject-10-moving-out/gesture_call-126-joints.bag",
        "subject-10-out/gesture_call-52-joints.bag",
        "subject-1-moving-out/gesture_call-116-joints.bag",
        "subject-1-moving-out/gesture_call-72-joints.bag",
        "subject-1-out/gesture_call-75-joints.bag",
        "subject-1-out/gesture_call-92-joints.bag",
        "subject-2-moving-out/gesture_call-123-joints.bag",
        "subject-3-out/gesture_call-50-joints.bag",
        "subject-4-moving-out/gesture_call-116-joints.bag",
        "subject-4-out/gesture_call-57-joints.bag",
        "subject-5-moving-out/gesture_call-186-joints.bag",
        "subject-5-moving-out/gesture_call-190-joints.bag",
        "subject-5-out/gesture_call-50-joints.bag",
        "subject-6-moving-out/gesture_call-110-joints.bag",
        "subject-6-out/gesture_call-56-joints.bag",
        "subject-7-moving-out/gesture_call-123-joints.bag",
        "subject-7-out/gesture_call-48-joints.bag",
        "subject-8-moving-out/gesture_call-92-joints.bag",
        "subject-8-out/gesture_call-53-joints.bag",
        "subject-9-moving-out/gesture_call-132-joints.bag",
        "subject-9-moving-out/gesture_call-156-joints.bag",
        "subject-9-out/gesture_call-78-joints.bag",
        "subject-9-out/gesture_call-99-joints.bag",
        ]

class Config():

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.parameters = {"called_robot": [True],
                            "seen_picking": [False],
                            "picker_speed": [0.8],
                            "target_picker": ["Picker02"],
                            "iterations": ["90"]}
        self.behaviours = {
                            # 1: {"type": "rosbag",
                            #     "label": "picking_berries",
                            #     "start": 0,
                            #     "duration": 3,
                            #     "filename": "/data/subject-1-out/picking_berries-34-joints.bag",
                            #     "topics": ["/human_actions"]
                            #     },
                            # 0: {"type": "rosbag",
                            #     "label": "call",
                            #     "start": 0,
                            #     "duration": 2,
                            #     "filename": "/data/subject-1-out/gesture_call-75-joints.bag",
                            #     "topics": ["/human_actions"]
                            #     },
                            0: {"type": "message",
                                "target": "picker_movement",
                                "message": "deliver"
                                }
                            }
        self.behaviour_times = sorted(self.behaviours.keys())
        self.termination_time = 20
        self.launch_files = ["/home/rasberry/catkin_ws/src/rasberry_hri/launch/hri_agent.launch", "/home/rasberry/catkin_ws/src/rasberry_hri/launch/picker_mover.launch"]
        # self.launch_files = ["/home/rasberry/catkin_ws/src/rasberry_hri/launch/picker_mover.launch"]
        # self.launch_files = ["/home/rasberry/catkin_ws/src/rasberry_hri/launch/hri_agent.launch"]
        # self.robot_pose = PoseWithCovarianceStamped()
        # self.robot_pose.pose.pose.position.x = 11.649
        # self.robot_pose.pose.pose.position.y = 4.62
        # self.robot_pose.pose.pose.position.z = 0

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
