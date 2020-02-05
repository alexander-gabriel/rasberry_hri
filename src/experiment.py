from threading import Thread

import rospy
import rosbag
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from marvelmind_nav.msg import hedge_pos_a

from bdi.robot_control import RobotControl



class Config(Thread):

    def __init__(self):
        self.parameters = {}
        self.picker_pose = hedge_pos_a()
        self.picker_pose.address = 2 #PICKER_ID
        # TODO set robot and picker starting locations
        self.picker_pose.x_m = 1.0
        self.picker_pose.y_m = 1.0

        self.robot_pose = PoseWithCovarianceStamped()
        self.robot_pose.position.x = 1.0
        self.robot_pose.position.y = 1.0
        self.robot_pose.position.z = 0.0


    def add_parameter(self, label, values):
        if not isinstance(values, list):
            values = [values]
        self.parameters[label] = values


    def get_parameter_set(self):
        try:
            return self.parameter_set
        except AttributeError:
            keys, values = zip(*self.parameters.items())
            self.parameter_set = [dict(zip(keys,v)) for v in itertools.product(*values)]
            return self.parameter_set


    def setup(self, robot_pose_publisher, picker_pose_publisher):
        # add config specific stuff to reasoning system, <- do this as defined by rosparameters inside the respective nodes instead
        robot_pose_publisher.publish(self.robot_pose)
        picker_pose_publisher.publish(self.picker_pose)


    def run(self):
        # TODO start required ros nodes, block until experiment has finished



class Experimenter:

    def __init__(self, robot_id):
        self.configs = []
        self.robot_id = robot_id
        self.robco = RobotControl(self.robot_id)
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        rospy.loginfo("EXP: Waiting for gazebo services")
        rospy.wait_for_service('/gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/reset_world')
        rospy.loginfo("EXP: Found gazebo services")
        self.robot_pose_publisher = rospy.Publisher('/{:}/initialpose'.format(robot_id), PoseWithCovarianceStamped, queue_size=10)
        self.picker_pose_publisher = rospy.Publisher('/hedge_pos_a', hedge_pos_a, queue_size=10)


    def reset(self):
        self.robco.cancel_movement()
        # ?? stop all running rosbags?
        self.reset_simulation()


    def run(self):
        for config in self.configs:
            for parameters in config.get_parameter_set():
                # this is one experiment run
                self.reset()
                for label, value in parameters:
                    rospy.set_param(label, value)
                config.setup(self.robot_pose_publisher, self.picker_pose_publisher)
                config.start()
                config.join()
