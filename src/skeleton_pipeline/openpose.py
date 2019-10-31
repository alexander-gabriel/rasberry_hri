import threading
import subprocess
from collections import deque
from time import sleep
import signal
import sys

import rospy
import rosgraph

from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Recognize
from image_recognition_msgs.msg import Recognitions




class Openpose(threading.Thread):

    RGB = True
    THERMAL = False

    def __init__(self, serviceProxy, callback):
        threading.Thread.__init__(self)
        self.callback = callback
        self.serviceProxy = serviceProxy
        # self.publisher = rospy.Publisher('/lcas/hri/joints/positions/raw', Recognitions, queue_size=10)
        self.died = False
        self.last_processed = Openpose.RGB
        self.latest_rgb = deque(maxlen=1)
        self.latest_thermal = deque(maxlen=1)
        signal.signal(signal.SIGINT, self.signal_handler)


    def signal_handler(self, sig, frame):
        self.died = True


    def run(self):
        while rosgraph.is_master_online() and not self.died:
            try:
                #self.last_processed = not self.last_processed
                if self.last_processed == Openpose.RGB and self.latest_rgb:
                    latest = self.latest_rgb.pop()
                    response = self.serviceProxy(latest)
                    self.callback(latest.header.stamp, "RGB", response)
                elif self.last_processed == Openpose.THERMAL and self.latest_thermal:
                    latest = self.latest_thermal.pop()
                    rospy.loginfo(latest.encoding)
                    response = self.interface(latest)
                    self.callback(latest.header.stamp, "THERMAL", response)
                else:
                    sleep(0.025)

            except rospy.ServiceException as exc:
                self.died = True
                print("Service did not process request: " + str(exc))


    # def send_message(self, response, category, timestamp):
    #     msg = Recognitions()
    #     msg.recognitions = response.recognitions
    #     msg.header.frame_id = category
    #     msg.header.stamp = timestamp
    #     self.publisher.publish(msg)
