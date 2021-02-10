#!/usr/bin/env python2
import os
import glob
from threading import Thread
import traceback

import numpy as np
import matplotlib
matplotlib.use('GTKCairo')
import matplotlib.pyplot as plt

import rospy
import rosbag

from rasberry_hri.msg import Action, Classification




SLEEP=0.01
PATH='/data'
classifications = []

def get_rosbag_from_file(filename, mode='r'):
    try:
        bag = rosbag.Bag(filename, mode)
        return bag
    except Exception as e:
        print("Failed to get rosbag topics info from file {:} with exception: '{:}'".format(filename, e))
        return None


class Player(Thread):

    def __init__(self, filename, topics=[], start_time=None):
        super(Player, self).__init__()
        self.bag = get_rosbag_from_file(filename)
        self.msgs = self.bag.read_messages(topics=topics)
        self.pubs = {}
        self.start_timestamp = start_time


    def run(self):
        for topic, msg, timestamp in self.msgs:
            # topic = "/human_actions_old"
            if topic not in self.pubs:
                self.pubs[topic] = rospy.Publisher(topic, msg.__class__, queue_size=10)
                rospy.sleep(1)
            if self.start_timestamp is None or timestamp > self.start_timestamp:
                self.pubs[topic].publish(msg)
                rospy.sleep(SLEEP)

    def close(self):
        self.bag.close()


def classification_callback(msg):
    classifications.append(msg)


def create_graph(filename, classifications, action_label):
    classifications = sorted(classifications, key=lambda c: c.header.stamp.to_sec())
    l = len(classifications)
    m = len(classifications[0].poses)
    n = len(classifications[0].poses[0].criteria)
    print("{}, {}, {}".format(l,m,n))
    t = []
    poses = {}
    # for pose_index in range(l):
    #     pose = list()
    #     for criterium_index in range(m):
    #         pose.append(list())
    #     poses.append(pose)
    #
    for classification in classifications:
        t.append(classification.header.stamp.to_sec())
        # for index in range(l):
        #     poses[index].append(classification.poses[index])

    for pose_index in range(m):
        for classification_index in range(l):
            for criterium_index in range(len(classifications[classification_index].poses[pose_index].criteria)):
                in_pose = classifications[classification_index].poses[pose_index]
                pose_label = in_pose.label
                if not pose_label in poses:
                    poses[pose_label] = {}
                pose = poses[pose_label]
                in_criterium = in_pose.criteria[criterium_index]
                code = in_criterium.code
                if not code in pose:
                    pose[code] = {"value": [], "limit": [], "error":[]}
                pose[code]["value"].append(in_criterium.value)
                pose[code]["limit"].append(in_criterium.limit)
                pose[code]["error"].append(in_criterium.error)

    for pose_index, (pose_label, pose) in enumerate(poses.items()):
        if action_label == pose_label:
        # if "gesture_stop" == pose_label or "neutral" == pose_label:
            fig = plt.figure(figsize=plt.figaspect(9.))
            l1 = None
            l2 = None
            l3 = None
            # fig,ax=plt.subplots(len(pose.keys()), 1, figsize=[])
            for criterium_index, (code, criterium) in enumerate(pose.items()):
                ax = fig.add_subplot(len(pose), 1, criterium_index+1)
                # plt.subplot(m*n, pose_index+1, criterium_index+1)
                # fig, ax = plt.subplots()
                ax.set_title("{:} - {:}".format(pose_label, code))
                ax.set_xlim((t[0], t[-1]))
                ax.set_ylabel("values")
                ax.set_xlabel("time")
                l1 = ax.plot(t, criterium["value"], label='value')
                l2 = ax.plot(t, criterium["limit"], label='limit')
                l3 = ax.plot(t, criterium["error"], label='error', linewidth=2)
                plt.legend()
                # ax.legend()



                # plt.setp(lines[0], linewidth=2)
                # plt.setp(lines[1], linewidth=2)
                # plt.setp(lines[2], linewidth=2)
                # fig.tight_layout()
                # axs[0].grid(True)
            # fig.tight_layout()
            fig.set_tight_layout(True)
            # fig.legend((l1, l2, l3), ('value', 'limit', 'error'), bbox_to_anchor=(1.04,1), loc="upper left")

            plt.margins(0.2)
            # plt.show()
            plt.savefig(filename)


# deliver_crate-23-joints.bag
# deposit_crate-31-joints.bag
# gesture_backward-72-joints.bag
# gesture_backward-88-joints.bag
# gesture_call-75-joints.bag
# gesture_call-92-joints.bag
# gesture_cancel-77-joints.bag
# gesture_cancel-96-joints.bag
# gesture_forward-69-joints.bag
# gesture_forward-83-joints.bag
# gesture_stop-81-joints.bag
# gesture_stop-99-joints.bag
# picking_berries-34-joints.bag
# pickup_crate-49-joints.bag
# return_crate-59-joints.bag
# turning-67-joints.bag
# walk_away-62-joints.bag
# walk_away_crate-26-joints.bag
# walk_towards-18-joints.bag
# walk_towards_crate-54-joints.bag

if __name__ == '__main__':
    normal_mode = False
    rospy.init_node("cla_exp")
    rospy.Subscriber("/pose_classification", Classification, classification_callback)
    ids = [1,2,3,4,5,6,7,8,9,10]
    sids = list()
    for id in ids:
        sids.append(str(id))
        sids.append("{}-moving".format(id))
    sids = ["1"]
    # open database
    for sid in sids:
        ## TODO: readjust
        source_folder = os.path.join(PATH, "subject-{:}-out".format(sid))
        print(source_folder)
        for filename in ["picking_berries-34-joints.bag"]: #os.listdir(source_folder):
            classifications = []
            try:
                print("starting with {}".format(filename))
                # recorder = Recorder(os.path.join(target_folder, "{:}-joints.bag".format(filename[:-4])), topic="/joints", klass=Action)
                ## TODO: readjust
                player = Player(os.path.join(source_folder,"{:}".format(filename)), topics=["/human_actions"], start_time=None)
                # player = Player(os.path.join(source_folder,"{:}".format(filename)), topics=["/camera/color/image_raw"], start_time=None)
                player.start()
                player.join()
                player.close()
                rospy.sleep(1)
            except Exception as err:
                print(traceback.format_exc())
            create_graph(os.path.join(source_folder,"{:}.png".format(filename[:-4])), classifications, filename.split("-")[0])
