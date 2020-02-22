#!/usr/bin/env python2
import os
import glob
from threading import Thread
import traceback

import rospy
import rosbag

from rasberry_hri.msg import Action

SLEEP=0.1 # 0.1 640x480, 0.05 320x240, 0.01 only action rec
PATH='/data'

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



class Recorder(object):
    def __init__(self, filename, topic, klass):
        super(Recorder, self).__init__()
        self.bag = get_rosbag_from_file(filename, 'w')
        self.subs = []
        self.topic = topic
        self.subs.append(rospy.Subscriber(topic, klass, self.callback))

    def close(self):
        for sub in self.subs:
            sub.unregister()
        self.bag.flush()
        self.bag.close()

    def callback(self, msg):
        self.bag.write(self.topic, msg, msg.header.stamp)



if __name__ == '__main__':
    rospy.init_node("play_record")
    ids = [1,2,3,4,5,6,7,8,9,10]
    sids = list()
    for id in ids:
        sids.append(str(id))
        sids.append("{}-moving".format(id))
    # sids = ["1"]
    # open database
    for sid in sids:
        ## TODO: readjust
        source_folder = os.path.join(PATH, "subject-{:}".format(sid))
        target_folder = os.path.join(PATH, "subject-{:}-out".format(sid))
        try:
            os.mkdir(target_folder)
        except:
            pass
        print(source_folder)
        for filename in os.listdir(source_folder):
            try:
                print("starting with {}".format(filename))
                recorder = Recorder(os.path.join(target_folder, "{:}-joints.bag".format(filename[:-4])), topic="/human_actions", klass=Action)
                ## TODO: readjust
                # player = Player(os.path.join(source_folder,"{:}".format(filename)), topics=["/human_actions"], start_time=None)
                player = Player(os.path.join(source_folder,"{:}".format(filename)), topics=["/camera/color/image_raw"], start_time=None)
                player.start()
                player.join()
                player.close()
                rospy.sleep(1)
                recorder.close()
            except Exception as err:
                print(traceback.format_exc())
