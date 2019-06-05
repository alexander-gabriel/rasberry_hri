#!/usr/bin/env python

import sys

import rospy
from image_recognition_msgs.msg import Recognitions
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rasberry_hri.msg import Joints, Joint, Pose, Log




class SkeletonLogger():

    def __init__(self):
        rospy.loginfo("SLS: Skeleton Logger Service starting")
        rospy.init_node('skeleton_logger_node', anonymous=False)

        self.pose = None
        self.angles = None
        self.angles_changed = False
        self.image = None
        self.image_changed = False

        self.record = False
        self.count = 0

        self.publisher = rospy.Publisher('/lcas/hri/log', Log, queue_size=10)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/lcas/hri/joints/angles", Joints, self.angles_callback)
        rospy.Subscriber("/lcas/hri/poses", Pose, self.pose_callback)
        rospy.Subscriber("/lcas/hri/snapshot", String, self.snapshot_callback)


    def send_maybe(self, header):
        if self.record and self.count < 90:
            if self.angles_changed and self.image_changed:
                outmsg = Log()
                outmsg.header = header
                outmsg.pose = self.pose
                outmsg.joints = self.angles
                outmsg.image = self.image
                self.publisher.publish(outmsg)
                
                self.angles_changed = False
                self.image_changed = False
                self.count += 1
                return True
            else:
                return False
        else:
            self.count = 0
            self.record = False
            return False


    def pose_callback(self, msg):
        self.pose = msg


    def angles_callback(self, msg):
        self.joints = msg
        self.angles_changed = True
        self.send_maybe(msg.header)
#

    def image_callback(self, msg):
        self.image = msg
        self.image_changed = True
        self.send_maybe(msg.header):


    def snapshot_callback(self, msg):
        self.record = True
