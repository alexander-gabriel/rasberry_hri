import sys
import json

import rospy
from openpose import Openpose

from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Recognize
from image_recognition_msgs.msg import Recognitions

from rasberry_hri.msg import Action, Command

from converter import Converter
from classifiers import MinimumDifferenceClassifier
from utils import get_angle_prototype, get_position_prototype, suppress



class ActionRecognition:


    def __init__(self):
        rospy.loginfo("ACR: ActionRecognition Node starting")
        self.action_publisher = rospy.Publisher('/human_actions', Action, queue_size=10)
        self.command_publisher = rospy.Publisher('/movement_control', Command, queue_size=10)
        self.picker = rospy.get_param("target_picker", "Picker02")
        self.converter = Converter()
        self.classifier = MinimumDifferenceClassifier()
        rospy.wait_for_service('recognize')
        self.interface = rospy.ServiceProxy('recognize', Recognize)
        rospy.loginfo("ACR: Waiting for OpenPose")
        self.openpose = Openpose(self.interface, self.openpose_callback)
        self.detection_count = rospy.get_param("detection_count", 2)
        self.cooldown = rospy.get_param("cooldown", -5)
        self.last_detected_count = {}
        camera = rospy.get_param("~camera", "/camera/color/image_raw")
        rospy.loginfo("ACR: Subscribing to {:}".format(camera))
        rospy.Subscriber(camera, Image, self.callback_rgb)
        # rospy.Subscriber("/camera/color/image_raw", Image, self.callback_rgb)
        # rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback_depth)
        # rospy.Subscriber("/camera/infra1/image_rect_raw", Image, self.callback_infra1)
        # rospy.Subscriber("/camera/infra2/image_rect_raw", Image, self.callback_infra2)

        # ZED camera
        # rospy.Subscriber("/zed/left/image_rect_color", Image, self.callback_rgb)

        # Thermal camera
        # rospy.Subscriber("/optris/thermal_image_view", Image, self.callback_thermal)
        rospy.loginfo("ACR: Initialization Complete")


    def openpose_callback(self, timestamp, source, response):
        if response.recognitions:
            self.converter.create_index_map(response.recognitions)
            angles = self.get_angles(response.recognitions)
            positions = self.get_positions(response.recognitions)
            with open("/home/rasberry/angles.log", 'w') as f:
                json.dump(angles, f)
            with open("/home/rasberry/positions.log", 'w') as f:
                json.dump(positions, f)
            action = self.get_action(angles, positions)
            with open("/home/rasberry/action.log", 'w') as f:
                json.dump(action, f)
            if action is not None:
                rospy.loginfo("ACR: Detected behavior '{}'".format(action))
                outmsg = Action()
                # outmsg.header.stamp = rospy.get_rostime()
                outmsg.header.stamp = timestamp
                outmsg.action = action
                #TODO: identify picker
                outmsg.id = self.picker
                self.action_publisher.publish(outmsg)
                # outmsg = Command()
                # outmsg.header.stamp = timestamp
                # outmsg.command = action
                # self.command_publisher.publish(outmsg)


    def get_positions(self, recognitions):

        # convert x/y offsets to values relative to some reference point (neck?)
        # adjust joint number and setup and labels
        model = get_position_prototype()
        for id in model.keys():
            with suppress(KeyError):
                model[id] = self.converter.get_position(id)
        return model


    def get_angles(self, recognitions):
        model = get_angle_prototype()
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
                model[id] = self.converter.get_angle1(id)
            except:
                pass
        # self.publisher.publish(outmsg)
        # if msg.header.frame_id == "RGB":
        #     self.model.add_sample(CATEGORY_JOINTS_OPENPOSE_2D_RGB, msg.header.stamp, model)
        # elif msg.header.frame_id == "THERMAL":
        #     self.model.add_sample(CATEGORY_JOINTS_OPENPOSE_2D_THERMAL, msg.header.stamp, model)
        # else:
        #     print("unknown category")
        #self.model.classify(inp)
        return model


    def get_action(self, angles, positions):
        action_label, error = self.classifier.classify(angles, positions)
        try:
            rospy.logdebug("ACR: Detected behavior '{}' with penalty {}, counts {}".format(action_label, error, self.last_detected_count[action_label]))
        except:
            rospy.logdebug("ACR: Detected behavior '{}' with penalty {}, counts {}".format(action_label, error, 0))
        if error < self.classifier.limit:
            if not action_label in self.last_detected_count:
                self.last_detected_count[action_label] = 1
            else:
                self.last_detected_count[action_label] +=  1
            if self.last_detected_count[action_label] == self.detection_count:
                self.last_detected_count[action_label] = self.cooldown
                return action_label
        else:
            self.last_detected_count = {}
            return None


    def callback_rgb(self, data):
        self.openpose.latest_rgb.append(data)


    def callback_thermal(self, data):
        self.openpose.latest_thermal.append(data)
