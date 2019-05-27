#!/usr/bin/env python
import sys
import threading
import subprocess

import roslaunch
import rospy
from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Recognize

class Openpose(threading.Thread):
    def __init__(self, service):
        threading.Thread.__init__(self)
        self.service = service
        self.died = False

    def run(self):
        try:
            resp = self.service(self.data)
            print(resp)
        except rospy.ServiceException:
            self.died = True



class SkeleteonExtractor:
    def __init__(self):
        rospy.init_node('skeleteon_extractor', anonymous=False)
        rospy.wait_for_service('recognize')
        self.interface = rospy.ServiceProxy('recognize', Recognize)
        self.service = Openpose(self.interface)
        self.count = 0

        rospy.Subscriber("/camera", Image, self.callback)

    def callback(self, data):
        try:
            if (self.service.died):
                rospy.signal_shutdown("openpose not found")
            if (self.count < 100000):
                if not self.service.isAlive():
                    self.service = Openpose(self.interface)
                    self.count = self.count + 1
                    self.service.data = data
                    self.service.start()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

if __name__ == '__main__':
    rospy.myargv(argv=sys.argv)
    proc = subprocess.Popen(["rosrun", "openpose_ros", "openpose_ros_node"])
    se = SkeleteonExtractor()
    rospy.spin()
    proc.kill()
