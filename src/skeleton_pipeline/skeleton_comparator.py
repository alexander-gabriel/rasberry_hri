import rospy

from rasberry_hri.msg import Joints, Joint, Pose, Command
from classifiers import MinimumDifferenceClassifier

from utils import get_model_prototype

class SkeletonComparator():

    def __init__(self):
        rospy.loginfo("SkelComp: Skeleton Comparator Service starting")
        rospy.init_node('skeleton_comparator', anonymous=False)
        topic = rospy.get_param('robot_control', '/rasberry/robot_control/move_base')
        rospy.loginfo("SkelComp: Subscribing to /lcas/hri/joints/angles")

        self.classifier = MinimumDifferenceClassifier()
        self.pose_publisher = rospy.Publisher('/lcas/hri/poses', Pose, queue_size=10)
        self.command_publisher = rospy.Publisher(topic, Command, queue_size=10)
        rospy.Subscriber("/lcas/hri/joints/angles", Joints, self.angle_callback)

    def angle_callback(self, msg):
        model = get_model_prototype()
        pose = dict()
        for joint in msg.joints:
            pose[joint.label] = joint.angle
        pose_label, error = self.classifier.classify(pose)
        if error < self.classifier.limit:
            outmsg = Pose()
            outmsg.header.stamp = msg.header.stamp
            outmsg.pose = pose_label
            self.pose_publisher.publish(outmsg)
            outmsg = Command()
            outmsg.header.stamp = msg.header.stamp
            outmsg.command = pose_label
            self.command_publisher.publish(outmsg)
