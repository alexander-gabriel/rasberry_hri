#!/usr/bin/env python

from threading import Thread

# import simdkalman
# import numpy as np
import rospy

from converter import Converter
from sensor_msgs.msg import RegionOfInterest
from image_recognition_msgs.msg import Recognitions, Recognition, CategoricalDistribution, CategoryProbability
from rasberry_hri.msg import Joints, Joint

from common.utils import get_model_prototype, mean

PUBLISHING_RATE = 30 #Hz
CATEGORY_JOINTS_OPENPOSE_2D_RGB = 3
CATEGORY_JOINTS_OPENPOSE_2D_THERMAL = 4






class Publisher(Thread):
    def __init__(self, model):
        Thread.__init__(self)
        self.model = model
        self.position_publisher = rospy.Publisher('/lcas/hri/joints/positions/stabilized', Recognitions, queue_size=10)
        #self.angle_publisher = rospy.Publisher('/lcas/people_detection/skeleton_model/angles', JointState, queue_size=10)
        self.rate = rospy.Rate(PUBLISHING_RATE)


    def run(self):
        while not rospy.is_shutdown():
            msg = Recognitions()
            msg.header.stamp = rospy.get_rostime()
            msg.recognitions = self.model.get_recognitions()
            self.position_publisher.publish(msg)
            self.rate.sleep()


class Model:

    def __init__(self, converter):
        self.converter = converter
        self.samples = dict()
        self.sampleSize = 10
        for label in self.converter.label_table.keys():
            self.samples[label] = list()


    def add_sample(self, label, data, timestamp):
        if not (data['X'] == 0 and data['Y'] == 0):
            self.samples[label].append((data, timestamp))


    def get_recognitions(self):
        recognitions = list()
        for label,samples in self.samples.items():
            if len(samples) > self.sampleSize:
                probability = []
                X = []
                Y = []
                for index in range(5):
                    index = -index-1
                    probability.append(samples[index][0]['probability'])
                    X.append(samples[index][0]['X'])
                    Y.append(samples[index][0]['Y'])
                probability = mean(probability)
                X = int(mean(X))
                Y = int(mean(Y))
                recognition = Recognition()

                cat_prob = CategoryProbability()
                cat_prob.label = label
                cat_prob.probability = probability

                recognition.categorical_distribution = CategoricalDistribution()
                recognition.categorical_distribution.probabilities.append(cat_prob)
                recognition.categorical_distribution.unknown_probability = 0.0

                recognition.roi = RegionOfInterest()
                recognition.roi.x_offset = X
                recognition.roi.y_offset = Y
                recognition.roi.height = 1
                recognition.roi.width = 1
                recognition.roi.do_rectify = False

                recognition.group_id = 0

                recognitions.append(recognition)
        return recognitions

    def get_distribution(self):
        iterator = iter(self.samples)
        samples = {}
        try:
            sample, timestamp = next(iterator)
            if not sample == None:
                for joint in sample.keys():
                    samples[joint] = [sample[joint]]
                cutoff_time = timestamp - COLLECTION_TIMEFRAME
                while timestamp > cutoff_time:
                    try:
                        sample, timestamp = next(iterator)
                        for joint in sample.keys():
                            samples[joint].append(sample[joint])
                    except StopIteration:
                        break
                model = get_probability_model_prototype()
                for joint in samples.keys():
                    model[joint] = [mean(samples[joint]), std(samples[joint])]
                return model
        except StopIteration:
            return None


class SkeletonStabilizer():

    def __init__(self):
        rospy.loginfo("SSS: Skeleton Stabilizer Service starting")
        rospy.init_node('skeleton_stabilizer_node', anonymous=False)
        self.converter = Converter()
        self.model = Model(self.converter)
        self.publisher = Publisher(self.model)
        self.publisher.start()

        rospy.loginfo("SSS: Subscribing to /lcas/hri/joints/positions/raw")
        rospy.Subscriber("/lcas/hri/joints/positions/raw", Recognitions, self.openPoseCallback)


    def openPoseCallback(self, msg):
        recognitions = self.converter.from_openpose(msg.recognitions)
        for label, joint in recognitions.items():
            # new_label = self.converter.from_openpose_labels(label)
            # if new_label is None:
                # rospy.logerror(label)
            self.model.add_sample(label, joint, msg.header.stamp)

        # convert x/y offsets to values relative to some reference point (neck?)
        # adjust joint number and setup and labels
        # model = get_model_prototype()
        # for id in ['Neck-Z', 'Upper-Spine-X', 'Mid-Spine-X', 'Lower-Spine-X']:
        #     try:
        #         model[id] = self.converter.get_angle2(id)
        #     except:
        #         pass
        #
        # for id in ['Right:Elbow-X', 'Upper-Spine-X', 'Mid-Spine-X', 'Lower-Spine-X', 'Left:Elbow-X', 'Right:Shoulder-X', 'Left:Shoulder-X', 'Right:Hip-X', 'Left:Hip-X', 'Right:Knee-X', 'Left:Knee-X']:
        #     try:
        #         model[id] = self.converter.get_angle1(id)
        #     except:
        #         pass
        # outmsg = Joints()
        # outmsg.header.stamp = msg.header.stamp
        # outmsg.source = msg.header.frame_id + "-LElbow"
        # outmsg.joints = []
        # for label,angle in model.items():
        #     joint = Joint()
        #     joint.label = label
        #     joint.angle = angle
        #     outmsg.joints.append(joint)
        # self.publisher.publish(outmsg)
        # if msg.header.frame_id == "RGB":
        #     self.model.add_sample(CATEGORY_JOINTS_OPENPOSE_2D_RGB, msg.header.stamp, model)
        # elif msg.header.frame_id == "THERMAL":
        #     self.model.add_sample(CATEGORY_JOINTS_OPENPOSE_2D_THERMAL, msg.header.stamp, model)
        # else:
        #     print("unknown category")
        #self.model.classify(inp)
