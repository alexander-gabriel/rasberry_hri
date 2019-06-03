import rospy

from rasberry_hri.msg import Joints, Joint, Pose, Command
from classifiers import MinimumDifferenceClassifier

from utils import get_model_prototype

class SkeletonComparator():

    def __init__(self):
        rospy.loginfo("SkelComp: Skeleton Comparator Service starting")
        rospy.init_node('skeleton_comparator_node', anonymous=False)
        topic = rospy.get_param('robot_control', '/rasberry/robot_control/move_base')

        self.classifier = MinimumDifferenceClassifier()
        self.pose_publisher = rospy.Publisher('/lcas/hri/poses', Pose, queue_size=10)
        self.command_publisher = rospy.Publisher(topic, Command, queue_size=10)

        rospy.loginfo("SkelComp: Subscribing to /lcas/hri/joints/angles")
        rospy.Subscriber("/lcas/hri/joints/angles", Joints, self.angle_callback)
        self.last_detected_count = {}


    def angle_callback(self, msg):
        model = get_model_prototype()
        pose = dict()
        for joint in msg.joints:
            pose[joint.label] = joint.angle
        pose_label, error = self.classifier.classify(pose)
        if error < self.classifier.limit:
            if not pose_label in self.last_detected_count:
                self.last_detected_count[pose_label] = 1
            else:
                self.last_detected_count[pose_label] +=  1
            if self.last_detected_count[pose_label] > 9:
                outmsg = Pose()
                outmsg.header.stamp = msg.header.stamp
                outmsg.pose = pose_label
                self.pose_publisher.publish(outmsg)
                outmsg = Command()
                outmsg.header.stamp = msg.header.stamp
                outmsg.command = pose_label
                self.command_publisher.publish(outmsg)

        else:
            self.last_detected_count = {}
