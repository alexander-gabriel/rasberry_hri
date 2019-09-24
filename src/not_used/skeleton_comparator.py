import rospy

from rasberry_hri.msg import Joints, Joint, Action, Command
from classifiers import MinimumDifferenceClassifier

from utils import get_model_prototype

class SkeletonComparator():

    def __init__(self):
        rospy.loginfo("SkelComp: Skeleton Comparator Service starting")
        rospy.init_node('skeleton_comparator_node', anonymous=False)
        topic = rospy.get_param('robot_control', '/lcas/hri/robot_control')

        self.classifier = MinimumDifferenceClassifier()
        self.action_publisher = rospy.Publisher('/lcas/hri/actions', Action, queue_size=10)
        self.command_publisher = rospy.Publisher(topic, Command, queue_size=10)

        rospy.loginfo("SkelComp: Subscribing to /lcas/hri/joints/angles")
        rospy.Subscriber("/lcas/hri/joints/angles", Joints, self.angle_callback)
        self.last_detected_count = {}


    def angle_callback(self, msg):
        model = get_model_prototype()
        action = dict()
        for joint in msg.joints:
            action[joint.label] = joint.angle
        action_label, error = self.classifier.classify(action)
        if error < self.classifier.limit:
            if not action_label in self.last_detected_count:
                self.last_detected_count[action_label] = 1
            else:
                self.last_detected_count[action_label] +=  1
            if self.last_detected_count[action_label] > 4:
                outmsg = Action()
                outmsg.header.stamp = msg.header.stamp
                outmsg.action = action_label
                outmsg.pose = msg.pose
                self.action_publisher.publish(outmsg)
                outmsg = Command()
                outmsg.header.stamp = msg.header.stamp
                outmsg.command = action_label
                self.command_publisher.publish(outmsg)

        else:
            self.last_detected_count = {}
