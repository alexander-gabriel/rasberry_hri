#!/usr/bin/env python
import sys


import rospy
from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Recognize

from openpose import Openpose


class SkeletonExtractor:

    def __init__(self):
        rospy.loginfo("SES: Skeleton Extractor Service starting")
        rospy.init_node('skeleton_extractor_node', anonymous=False)
        rospy.loginfo("SES: Waiting for openpose_ros_node")
        rospy.wait_for_service('recognize')
        self.interface = rospy.ServiceProxy('recognize', Recognize)
        self.service = Openpose(self.interface)


        camera = rospy.get_param("~camera", "/camera/color/image_raw")
        rospy.loginfo("SES: Subscribing to {:}".format(camera))
        rospy.Subscriber(camera, Image, self.callback_rgb)
        # rospy.Subscriber("/camera/color/image_raw", Image, self.callback_rgb)
        # rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback_depth)
        # rospy.Subscriber("/camera/infra1/image_rect_raw", Image, self.callback_infra1)
        # rospy.Subscriber("/camera/infra2/image_rect_raw", Image, self.callback_infra2)

        # ZED camera
        # rospy.Subscriber("/zed/left/image_rect_color", Image, self.callback_rgb)

        # Thermal camera
        # rospy.Subscriber("/optris/thermal_image_view", Image, self.callback_thermal)
        rospy.loginfo("SES: Initialization Complete")


    def callback_rgb(self, data):
        self.service.latest_rgb.append(data)


    def callback_thermal(self, data):
        self.service.latest_thermal.append(data)
