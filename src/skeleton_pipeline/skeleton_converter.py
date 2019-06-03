#!/usr/bin/env python

import sys

import rospy
from image_recognition_msgs.msg import Recognitions
from rasberry_hri.msg import Joints, Joint

from converter import Converter
from utils import get_model_prototype

CATEGORY_JOINTS_OPENPOSE_2D_RGB = 3
CATEGORY_JOINTS_OPENPOSE_2D_THERMAL = 4


class SkeletonConverter():

    def __init__(self):
        self.converter = Converter()
        rospy.loginfo("SCS: Skeleton Converter Service starting")
        rospy.init_node('skeleton_converter_node', anonymous=False)

        self.publisher = rospy.Publisher('/lcas/hri/joints/angles', Joints, queue_size=10)
        rospy.loginfo("SCS: Subscribing to /lcas/hri/joint/positions/raw")
        rospy.Subscriber("/lcas/hri/joints/positions/raw", Recognitions, self.openpose_callback)


    def openpose_callback(self, msg):
        self.converter.create_index_map(msg.recognitions)
        # convert x/y offsets to values relative to some reference point (neck?)
        # adjust joint number and setup and labels
        model = get_model_prototype()
        for id in ['Neck-Z', 'Upper-Spine-X', 'Mid-Spine-X', 'Lower-Spine-X']:
            try:
                model[id] = self.converter.get_angle2(id)
            except:
                pass

        for id in ['Right:Elbow-X', 'Upper-Spine-X', 'Mid-Spine-X', 'Lower-Spine-X', 'Right:Shoulder-X', 'Right:Hip-X', 'Right:Knee-X']:
            try:
                model[id] = self.converter.get_angle1(id)
            except:
                pass
        for id in ['Left:Elbow-X', 'Left:Shoulder-X', 'Left:Hip-X', 'Left:Knee-X']:
            try:
                model[id] = -self.converter.get_angle1(id)
            except:
                pass
        outmsg = Joints()
        outmsg.header.stamp = msg.header.stamp
        outmsg.source = msg.header.frame_id
        outmsg.joints = []
        for label,angle in model.items():
            joint = Joint()
            joint.label = label
            joint.angle = angle
            outmsg.joints.append(joint)
        self.publisher.publish(outmsg)
        # if msg.header.frame_id == "RGB":
        #     self.model.add_sample(CATEGORY_JOINTS_OPENPOSE_2D_RGB, msg.header.stamp, model)
        # elif msg.header.frame_id == "THERMAL":
        #     self.model.add_sample(CATEGORY_JOINTS_OPENPOSE_2D_THERMAL, msg.header.stamp, model)
        # else:
        #     print("unknown category")
        #self.model.classify(inp)
