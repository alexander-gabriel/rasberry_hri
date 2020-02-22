import sys
import json
import time

from math import pi

import rospy
from openpose import Openpose

from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Recognize
from image_recognition_msgs.msg import Recognitions

from rasberry_hri.msg import Action, Command, Joint, Classification

from converter import Converter
from classifiers import MinimumDifferenceClassifier
from utils import get_angle_prototype, get_position_prototype, suppress



class ActionRecognition:


    def __init__(self, normal_mode=False):
        self.normal_mode = normal_mode
        rospy.loginfo("ACR: ActionRecognition Node starting")
        if self.normal_mode:
            self.action_publisher = rospy.Publisher('/human_actions', Action, queue_size=10)
        else:
            self.joint_publisher = rospy.Publisher('/joints', Action, queue_size=10)
        self.command_publisher = rospy.Publisher('/movement_control', Command, queue_size=10)
        self.classification_publisher = rospy.Publisher('/pose_classification', Classification, queue_size=10)
        self.picker = rospy.get_param("target_picker", "Picker02")
        self.converter = Converter()
        self.classifier = MinimumDifferenceClassifier()
        if self.normal_mode:
            rospy.wait_for_service('recognize')
            self.interface = rospy.ServiceProxy('recognize', Recognize)
            rospy.loginfo("ACR: Waiting for OpenPose")
            self.openpose = Openpose(self.interface, self.openpose_callback)
        self.detection_count = rospy.get_param("detection_count", 2)
        self.cooldown = rospy.get_param("cooldown", -5)
        self.last_detected_count = {}
        camera = rospy.get_param("~camera", "/camera/color/image_raw")
        rospy.loginfo("ACR: Subscribing to {:}".format(camera))
        if self.normal_mode:
            rospy.Subscriber(camera, Image, self.callback_rgb)
        else:
            rospy.Subscriber('/human_actions', Action, self.action_callback)



        # rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback_depth)
        # rospy.Subscriber("/camera/infra1/image_rect_raw", Image, self.callback_infra1)
        # rospy.Subscriber("/camera/infra2/image_rect_raw", Image, self.callback_infra2)

        # ZED camera
        # rospy.Subscriber("/zed/left/image_rect_color", Image, self.callback_rgb)

        # Thermal camera
        # rospy.Subscriber("/optris/thermal_image_view", Image, self.callback_thermal)
        rospy.loginfo("ACR: Initialization Complete")

    def run(self):
        if self.normal_mode:
            self.openpose.run()
        else:
            rospy.spin()


    def openpose_callback(self, timestamp, source, response):
        if response.recognitions:
            self.converter.create_index_map(response.recognitions)
            angles = self.get_angles(response.recognitions)
            positions = self.get_positions(response.recognitions)
            with open("/home/rasberry/angles.log", 'w') as f:
                json.dump(angles, f)
            with open("/home/rasberry/positions.log", 'w') as f:
                json.dump(positions, f)
            action, classification  = self.get_action(angles, positions)
            classification.header.stamp = timestamp
            self.classification_publisher.publish(classification)
            with open("/home/rasberry/action.log", 'w') as f:
                json.dump(action, f)
            if action is not None:
                rospy.loginfo("ACR: Detected behavior '{}'".format(action))
            outmsg = Action()
            outmsg.header.stamp = timestamp
            outmsg.action = action or ""
            outmsg.person = self.picker
            # outmsg.header.stamp = rospy.get_rostime()
            outmsg.joints = []
            for key in angles.keys():
                key = key[:-2]
                joint = Joint()
                joint.label = key
                try:
                    joint.angle = angles[key+"-X"]
                except:
                    pass
                try:
                    joint.position.x = positions[key+"-X"]
                    joint.position.y = positions[key+"-Y"]
                except:
                    pass
                outmsg.joints.append(joint)
                #TODO: identify picker

            self.action_publisher.publish(outmsg)
                # outmsg = Command()
                # outmsg.header.stamp = timestamp
                # outmsg.command = action
                # self.command_publisher.publish(outmsg)


    def action_callback(self, msg):
        start_time = time.time()
        positions = {} #get_position_prototype()
        angles = {} #get_angle_prototype()
        for joint in msg.joints:
            angle_key = "{:}-X".format(joint.label)
            pos_x_key = "{:}-X".format(joint.label)
            pos_y_key = "{:}-Y".format(joint.label)
            angles[angle_key] = joint.angle
            positions[pos_x_key] = joint.position.x
            positions[pos_y_key] = joint.position.y
        # rospy.loginfo(angles)
        # sys.exit(0)
        with open("/home/rasberry/angles.log", 'w') as f:
            json.dump(angles, f)
        with open("/home/rasberry/positions.log", 'w') as f:
            json.dump(positions, f)
        action, classification = self.get_action(angles, positions)
        classification.header.stamp = msg.header.stamp
        self.classification_publisher.publish(classification)
        with open("/home/rasberry/action.log", 'w') as f:
            json.dump(action, f)
        if action is not None:
            rospy.loginfo("ACR: Detected behavior '{}'".format(action))
        outmsg = Action()
        outmsg.header.stamp = msg.header.stamp
        outmsg.action = action or ""
        outmsg.person = self.picker
        # outmsg.header.stamp = rospy.get_rostime()
        outmsg.joints = []
        for key in angles.keys():
            key = key[:-3]

            joint = Joint()
            joint.label = key
            try:
                joint.angle = angles[key+"-X"]
            except:
                pass
            try:
                joint.position.x = positions[key+"-X"]
                joint.position.y = positions[key+"-Y"]
            except:
                pass
            outmsg.joints.append(joint)
            #TODO: identify picker
        # rospy.logwarn("ACR: action recognition took {:.4f}s".format(time.time()-start_time))
        # self.joint_publisher.publish(outmsg)



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
        tuple, classification = self.classifier.classify(angles, positions)
        action_label = tuple[0]
        error = tuple[1]
        try:
            rospy.logdebug("ACR: Detected behavior '{}' with penalty {}, counts {}".format(action_label, error, self.last_detected_count[action_label]))
        except:
            rospy.logdebug("ACR: Detected behavior '{}' with penalty {}, counts {}".format(action_label, error, 0))
        if error < self.classifier.limit:
            if not action_label in self.last_detected_count:
                self.last_detected_count[action_label] = 1
            else:
                self.last_detected_count[action_label] +=  1
            ## TODO: readjust
            #if self.last_detected_count[action_label] == self.detection_count:
            self.last_detected_count[action_label] = self.cooldown
            return (action_label, classification)
        else:
            self.last_detected_count = {}
            return (None, classification)


    def callback_rgb(self, data):
        self.openpose.latest_rgb.append(data)


    def callback_thermal(self, data):
        self.openpose.latest_thermal.append(data)
