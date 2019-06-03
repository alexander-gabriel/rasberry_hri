
from numpy import arctan2, abs
import numpy as np

from filter import LimbFilter, PositionFilter

class Converter:

    def __init__(self):
        self.angle_joints = {'Right:Elbow-X' : ("RWrist", "RElbow", "RShoulder"),
                             'Left:Elbow-X' : ("LWrist", "LElbow", "LShoulder"),
                             'Right:Shoulder-X' : ("Neck", "RShoulder", "RElbow"),
                             'Left:Shoulder-X' : ("Neck", "LShoulder", "LElbow"),
                             'Right:Knee-X' : ("RAnkle", "RKnee", "RHip"),
                             'Left:Knee-X' : ("LAnkle", "LKnee", "LHip"),
                             'Right:Hip-X' : ("RKnee", "RHip", "LHip"),
                             'Left:Hip-X' : ("LKnee", "LHip", "RHip"),
                             'Upper-Spine-X' : ("RHip", "LHip", "Neck"),
                             'Mid-Spine-X' : ("RHip", "LHip", "Neck"),
                             'Lower-Spine-X' : ("RHip", "LHip", "Neck"),
                             'Neck-Z' : ("REar", "LEar", "Neck")}

        self.label_table = {'RBigToe': 'Right:BigToe',
                                'LBigToe': 'Left:BigToe',
                                'RSmallToe': 'Right:SmallToe',
                                'LSmallToe': 'Left:SmallToe',
                                'RHeel': 'Right:Heel',
                                'LHeel': 'Left:Heel',
                                'RAnkle': 'Right:Ankle',
                                'LAnkle': 'Left:Ankle',
                                'RKnee': 'Right:Knee',
                                'LKnee': 'Left:Knee',
                                'RHip': 'Right:Hip',
                                'MidHip': 'Mid:Hip',
                                'LHip': 'Left:Hip',
                                'Neck': 'Neck',
                                'RWrist': 'Right:Wrist',
                                'LWrist': 'Left:Wrist',
                                'RElbow': 'Right:Wrist',
                                'LElbow': 'left:Wrist',
                                'RShoulder': 'Right:Shoulder',
                                'LShoulder': 'Left:Shoulder',
                                'Nose': 'Nose',
                                'REye': 'Right:Eye',
                                'LEye': 'Left:Eye',
                                'REar': 'Right:Ear',
                                'LEar': 'Left:Ear'}
        self.limb_filter = LimbFilter(0.5)
        self.position_filter = PositionFilter()


    def create_index_map(self, recognitions):
        self.index_map = {}
        self.recognitions = recognitions
        for index, entry in enumerate(self.recognitions):
            self.index_map[entry.categorical_distribution.probabilities[0].label] = index


    def X(self, label):
        return self.recognitions[self.index_map[label]].roi.x_offset


    def Y(self, label):
        return self.recognitions[self.index_map[label]].roi.y_offset


    def get_angle1(self, joint_name):
        joints = self.angle_joints[joint_name]
        v0 = np.array([self.X(joints[0]), self.Y(joints[0])]) - np.array([self.X(joints[1]), self.Y(joints[1])])
        v1 = np.array([self.X(joints[2]), self.Y(joints[2])]) - np.array([self.X(joints[1]), self.Y(joints[1])])

        angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
        return np.degrees(angle)


    def get_position(self, joint_name, model):
        pass


    def get_angle2(self, joint_name):
        joints = self.angle_joints[joint_name]
        midX = (self.X(joints[0]) - self.X(joints[1])) / 2.0
        midY = (self.Y(joints[0]) - self.Y(joints[1])) / 2.0
        v0 = np.array([self.X(joints[0]), self.Y(joints[0])]) - np.array([midX, midY])
        v1 = np.array([self.X(joints[2]), self.Y(joints[2])]) - np.array([midX, midY])

        angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
        return np.degrees(angle)


    def get_angle3(self, joint_name):
        joints = self.angle_joints[joint_name]
        aX = self.X(joints[0]) - self.X(joints[1])
        bX = self.X(joints[2]) - self.X(joints[1])
        cX = aX - bX
        aY = self.Y(joints[0]) - self.Y(joints[1])
        bY = self.Y(joints[2]) - self.Y(joints[1])
        cY = aY - bY
        return arctan2(-cY,cX)


    def get_angle4(self, joint_name):
        joints = self.angle_joints[joint_name]
        midX = (self.X(joints[0]) - self.X(joints[1]))/ 2.0
        midY = (self.Y(joints[0]) - self.Y(joints[1]))/2.0
        cX = self.X(joints[2]) - midX
        cY = self.Y(joints[2]) - midY
        return arctan2(-cY,cX)


    def from_openpose(self, recognitions):
        joints = {}
        recognitions = self.limb_filter.filter(recognitions)
        recognitions = self.position_filter.filter(recognitions)
        for entry in recognitions:
            label = entry.categorical_distribution.probabilities[0].label
            probability = entry.categorical_distribution.probabilities[0].probability
            X = entry.roi.x_offset
            Y = entry.roi.y_offset
            joints[label] = {}
            joints[label]['probability'] = probability
            joints[label]['X'] = X
            joints[label]['Y'] = Y
            joints[label]['Z'] = -1
        return joints


    def from_openpose_labels(self, label):
        return self.label_table[label]
