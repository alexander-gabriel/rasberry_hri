import sys
import rospy
from openpose import Openpose

from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Recognize
from image_recognition_msgs.msg import Recognitions

from rasberry_hri.msg import Action, Command

from converter import Converter
from classifiers import MinimumDifferenceClassifier
from utils import get_model_prototype



class ActionRecognition:


    def __init__(self):

        topic = rospy.get_param('robot_control', '/lcas/hri/robot_control')
        self.converter = Converter()
        self.classifier = MinimumDifferenceClassifier()
        self.interface = rospy.ServiceProxy('recognize', Recognize)
        rospy.loginfo("SES: Waiting for openpose_ros_node")
        # rospy.wait_for_service('recognize')
        self.service = Openpose(self.interface, self.openpose_callback)

        self.last_detected_count = {}

        rospy.loginfo("SES: ActionRecognition Node starting")
        rospy.init_node('action_recognition_node', anonymous=False)
        camera = rospy.get_param("~camera", "/camera/color/image_raw")
        self.action_publisher = rospy.Publisher('/lcas/hri/actions', Action, queue_size=10)
        self.command_publisher = rospy.Publisher(topic, Command, queue_size=10)



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


    def openpose_callback(self, timestamp, source, response):
        joints = self.get_angles(response.recognitions)
        action = get_action(joints)
        if action is not None:
            outmsg = Action()
            outmsg.header.stamp = timestamp
            outmsg.action = action
            outmsg.pose = msg.pose
            self.action_publisher.publish(outmsg)
            outmsg = Command()
            outmsg.header.stamp = timestamp
            outmsg.command = action
            self.command_publisher.publish(outmsg)


    def get_angles(self, recognitions):
        self.converter.create_index_map(recognitions)
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
                model[id] = self.converter.get_angle1(id)
            except:
                pass
        joints = []
        for label,angle in model.items():
            joints.append(joint)
        # self.publisher.publish(outmsg)
        # if msg.header.frame_id == "RGB":
        #     self.model.add_sample(CATEGORY_JOINTS_OPENPOSE_2D_RGB, msg.header.stamp, model)
        # elif msg.header.frame_id == "THERMAL":
        #     self.model.add_sample(CATEGORY_JOINTS_OPENPOSE_2D_THERMAL, msg.header.stamp, model)
        # else:
        #     print("unknown category")
        #self.model.classify(inp)
        return joints


    def get_action(self, joints):
        model = get_model_prototype()
        action = dict()
        for joint in joints:
            action[joint.label] = joint.angle
        action_label, error = self.classifier.classify(action)
        if error < self.classifier.limit:
            if not action_label in self.last_detected_count:
                self.last_detected_count[action_label] = 1
            else:
                self.last_detected_count[action_label] +=  1
            if self.last_detected_count[action_label] > 4:
                return action_label
        else:
            self.last_detected_count = {}
            return None


    def callback_rgb(self, data):
        self.service.latest_rgb.append(data)


    def callback_thermal(self, data):
        self.service.latest_thermal.append(data)
