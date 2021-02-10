#!/usr/bin/env python2
import os
import glob
from threading import Thread
import traceback

import rospy
import rosbag

from sensor_msgs.msg import Image

SLEEP=0.1 # 0.1 640x480, 0.05 320x240, 0.01 only action rec


data = {
    "approach with crate": {
        1: {
            "standing": (
                "/subject-1/walk_towards_crate-54",
                2,
                5,
            ),
            "moving": (
                "/subject-1-moving/walk_towards_crate-53",
                2,
                5,
            ),
        },
        2: {
            "standing": (
                "/subject-2/walk_towards_crate-27",
                2,
                5,
            ),
            "moving": (
                "/subject-2-moving/walk_towards_crate-28",
                2,
                5,
            ),
        },
        3: {
            "standing": (
                "/subject-3/walk_towards_crate-30",
                2,
                5,
            ),
            "moving": (
                "/subject-3-moving/walk_towards_crate-43",
                2,
                5,
            ),
        },
        4: {
            "standing": (
                "/subject-4/walk_towards_crate-34",
                2,
                5,
            ),
            "moving": (
                "/subject-4-moving/walk_towards_crate-47",
                2,
                5,
            ),
        },
        5: {
            "standing": (
                "/subject-5/walk_towards_crate-28",
                2,
                5,
            ),
            "moving": (
                "/subject-5-moving/walk_towards_crate-132",
                2,
                5,
            ),
        },
        6: {
            "standing": (
                "/subject-6/walk_towards_crate-37",
                2,
                5,
            ),
            "moving": (
                "/subject-6-moving/walk_towards_crate-35",
                2,
                5,
            ),
        },
        7: {
            "standing": (
                "/subject-7/walk_towards_crate-28",
                2,
                5,
            ),
            "moving": (
                "/subject-7-moving/walk_towards_crate-67",
                2,
                5,
            ),
        },
        8: {
            "standing": (
                "/subject-8/walk_towards_crate-30",
                2,
                5,
            ),
            "moving": (
                "/subject-8-moving/walk_towards_crate-44",
                2,
                5,
            ),
        },
        9: {
            "standing": (
                "/subject-9/walk_towards_crate-54",
                2,
                5,
            ),
            "moving": (
                "/subject-9-moving/walk_towards_crate-69",
                2,
                5,
            ),
        },
        10: {
            "standing": (
                "/subject-10/walk_towards_crate-30",
                2,
                5,
            ),
            "moving": (
                "/subject-10-moving/walk_towards_crate-41",
                2,
                5,
            ),
        },
    },
    "approach without crate": {
        1: {
            "standing": (
                "/subject-1/walk_towards-18",
                2,
                5,
            ),
            "moving": (
                "/subject-1-moving/walk_towards-25",
                2,
                5,
            ),
        },
        2: {
            "standing": (
                "/subject-2/walk_towards-1",
                2,
                5,
            ),
            "moving": (
                "/subject-2-moving/walk_towards-2",
                2,
                5,
            ),
        },
        3: {
            "standing": (
                "/subject-3/walk_towards-1",
                2,
                5,
            ),
            "moving": (
                "/subject-3-moving/walk_towards-11",
                2,
                5,
            ),
        },
        4: {
            "standing": (
                "/subject-4/walk_towards-3",
                2,
                5,
            ),
            "moving": (
                "/subject-4-moving/walk_towards-9",
                2,
                5,
            ),
        },
        5: {
            "standing": (
                "/subject-5/walk_towards-1",
                2,
                5,
            ),
            "moving": (
                "/subject-5-moving/walk_towards-10",
                2,
                5,
            ),
        },
        6: {
            "standing": (
                "/subject-6/walk_towards-1",
                2,
                5,
            ),
            "moving": (
                "/subject-6-moving/walk_towards-2",
                2,
                5,
            ),
        },
        7: {
            "standing": (
                "/subject-7/walk_towards-2",
                2,
                5,
            ),
            "moving": (
                "/subject-7-moving/walk_towards-36",
                2,
                5,
            ),
        },
        8: {
            "standing": (
                "/subject-8/walk_towards-2",
                2,
                5,
            ),
            "moving": (
                "/subject-8-moving/walk_towards-9",
                2,
                5,
            ),
        },
        9: {
            "standing": (
                "/subject-9/walk_towards-0",
                2,
                5,
            ),
            "moving": (
                "/subject-9-moving/walk_towards-11",
                2,
                5,
            ),
        },
        10: {
            "standing": (
                "/subject-10/walk_towards-0",
                2,
                5,
            ),
            "moving": (
                "/subject-10-moving/walk_towards-5",
                2,
                5,
            ),
        },
    },
    "leave with crate": {
        1: {
            "standing": (
                "/subject-1/walk_away_crate-26",
                2,
                5,
            ),
            "moving": (
                "/subject-1-moving/walk_away_crate-32",
                2,
                5,
            ),
        },
        2: {
            "standing": (
                "/subject-2/walk_away_crate-9",
                2,
                5,
            ),
            "moving": (
                "/subject-2-moving/walk_away_crate-7",
                2,
                5,
            ),
        },
        3: {
            "standing": (
                "/subject-3/walk_away_crate-8",
                2,
                5,
            ),
            "moving": (
                "/subject-3-moving/walk_away_crate-19",
                2,
                5,
            ),
        },
        4: {
            "standing": (
                "/subject-4/walk_away_crate-10",
                2,
                5,
            ),
            "moving": (
                "/subject-4-moving/walk_away_crate-17",
                2,
                5,
            ),
        },
        5: {
            "standing": (
                "/subject-5/walk_away_crate-8",
                2,
                5,
            ),
            "moving": (
                "/subject-5-moving/walk_away_crate-18",
                2,
                5,
            ),
        },
        6: {
            "standing": (
                "/subject-6/walk_away_crate-9",
                2,
                5,
            ),
            "moving": (
                "/subject-6-moving/walk_away_crate-11",
                2,
                5,
            ),
        },
        7: {
            "standing": (
                "/subject-7/walk_away_crate-9",
                2,
                5,
            ),
            "moving": (
                "/subject-7-moving/walk_away_crate-43",
                2,
                5,
            ),
        },
        8: {
            "standing": (
                "/subject-8/walk_away_crate-9",
                2,
                5,
            ),
            "moving": (
                "/subject-8-moving/walk_away_crate-19",
                2,
                5,
            ),
        },
        9: {
            "standing": (
                "/subject-9/walk_away_crate-11",
                2,
                5,
            ),
            "moving": (
                "/subject-9-moving/walk_away_crate-27",
                2,
                5,
            ),
        },
        10: {
            "standing": (
                "/subject-10/walk_away_crate-8",
                2,
                5,
            ),
            "moving": (
                "/subject-10-moving/walk_away_crate-17",
                2,
                5,
            ),
        },
    },
    "leave without crate": {
        1: {
            "standing": (
                "/subject-1/walk_away-62",
                2,
                5,
            ),
            "moving": (
                "/subject-1-moving/walk_away-60",
                2,
                5,
            ),
        },
        2: {
            "standing": (
                "/subject-1/walk_away-62",
                2,
                5,
            ),
            "moving": (
                "/subject-2-moving/walk_away-35",
                2,
                5,
            ),
        },
        3: {
            "standing": (
                "/subject-3/walk_away-37",
                2,
                5,
            ),
            "moving": (
                "/subject-3-moving/walk_away-52",
                2,
                5,
            ),
        },
        4: {
            "standing": (
                "/subject-4/walk_away-43",
                2,
                5,
            ),
            "moving": (
                "/subject-4-moving/walk_away-55",
                2,
                5,
            ),
        },
        5: {
            "standing": (
                "/subject-5/walk_away-36",
                2,
                5,
            ),
            "moving": (
                "/subject-5-moving/walk_away-145",
                2,
                5,
            ),
        },
        6: {
            "standing": (
                "/subject-6/walk_away-46",
                2,
                5,
            ),
            "moving": (
                "/subject-6-moving/walk_away-42",
                2,
                5,
            ),
        },
        7: {
            "standing": (
                "/subject-7/walk_away-36",
                2,
                5,
            ),
            "moving": (
                "/subject-7-moving/walk_away-76",
                2,
                5,
            ),
        },
        8: {
            "standing": (
                "/subject-8/walk_away-38",
                2,
                5,
            ),
            "moving": (
                "/subject-8-moving/walk_away-57",
                2,
                5,
            ),
        },
        9: {
            "standing": (
                "/subject-9/walk_away-65",
                2,
                5,
            ),
            "moving": (
                "/subject-9-moving/walk_away-79",
                2,
                5,
            ),
        },
        10: {
            "standing": (
                "/subject-10/walk_away-40",
                2,
                5,
            ),
            "moving": (
                "/subject-10-moving/walk_away-51",
                2,
                5,
            ),
        },
    },
    "pick berries": {
        1: {
            "standing": (
                "/subject-1/picking_berries-34",
                1,
                3,
            ),
            "moving": (
                "/subject-1-moving/picking_berries-39",
                5,
                3,
            ),
        },
        2: {
            "standing": (
                "/subject-2/picking_berries-14",
                1,
                3,
            ),
            "moving": (
                "/subject-2-moving/picking_berries-10",
                1,
                3,
            ),
        },
        3: {
            "standing": (
                "/subject-3/picking_berries-16",
                5,
                3,
            ),
            "moving": (
                "/subject-3-moving/picking_berries-24",
                1,
                3,
            ),
        },
        4: {
            "standing": (
                "/subject-4/picking_berries-20",
                1,
                3,
            ),
            "moving": (
                "/subject-4-moving/picking_berries-25",
                1,
                3,
            ),
        },
        5: {
            "standing": (
                "/subject-5/picking_berries-15",
                1,
                3,
            ),
            "moving": (
                "/subject-5-moving/picking_berries-23",
                1,
                3,
            ),
        },
        6: {
            "standing": (
                "/subject-6/picking_berries-17",
                1,
                3,
            ),
            "moving": (
                "/subject-6-moving/picking_berries-17",
                3,
                3,
            ),
        },
        7: {
            "standing": (
                "/subject-7/picking_berries-17",
                1,
                3,
            ),
            "moving": (
                "/subject-7-moving/picking_berries-49",
                1,
                3,
            ),
        },
        8: {
            "standing": (
                "/subject-8/picking_berries-16",
                1,
                3,
            ),
            "moving": (
                "/subject-8-moving/picking_berries-24",
                3,
                3,
            ),
        },
        9: {
            "standing": (
                "/subject-9/picking_berries-22",
                3,
                3,
            ),
            "moving": (
                "/subject-9-moving/picking_berries-32",
                9,
                3,
            ),
        },
        10: {
            "standing": (
                "/subject-10/picking_berries-18",
                2,
                3,
            ),
            "moving": (
                "/subject-10-moving/picking_berries-23",
                1,
                3,
            ),
        },
    },
    "call robot": {
        1: {
            "standing": (
                "/subject-1/gesture_call-75",
                0,
                2,
            ),
            "moving": (
                "/subject-1-moving/gesture_call-116",
                3,
                2,
            ),
        },
        2: {
            "standing": (
                "/subject-1/gesture_call-92",
                1,
                2,
            ),
            "moving": (
                "/subject-2-moving/gesture_call-123",
                1,
                2,
            ),
        },
        3: {
            "standing": (
                "/subject-3/gesture_call-50",
                0,
                2,
            ),
            "moving": (
                "/subject-1-moving/gesture_call-116",
                3,
                2,
            ),
        },
        4: {
            "standing": (
                "/subject-4/gesture_call-57",
                1,
                2,
            ),
            "moving": (
                "/subject-4-moving/gesture_call-116",
                1,
                2,
            ),
        },
        5: {
            "standing": (
                "/subject-5/gesture_call-50",
                1,
                2,
            ),
            "moving": (
                "/subject-5-moving/gesture_call-190",
                6,
                2,
            ),
        },
        6: {
            "standing": (
                "/subject-6/gesture_call-56",
                1,
                2,
            ),
            "moving": (
                "/subject-6-moving/gesture_call-110",
                1,
                2,
            ),
        },
        7: {
            "standing": (
                "/subject-7/gesture_call-48",
                1,
                2,
            ),
            "moving": (
                "/subject-7-moving/gesture_call-123",
                5,
                2,
            ),
        },
        8: {
            "standing": (
                "/subject-8/gesture_call-53",
                1,
                2,
            ),
            "moving": (
                "/subject-8-moving/gesture_call-92",
                1,
                2,
            ),
        },
        9: {
            "standing": (
                "/subject-9/gesture_call-99",
                1,
                2,
            ),
            "moving": (
                "/subject-9-moving/gesture_call-156",
                1,
                2,
            ),
        },
        10: {
            "standing": (
                "/subject-10/gesture_call-52",
                1,
                2,
            ),
            "moving": (
                "/subject-10-moving/gesture_call-126",
                1,
                2,
            ),
        },
    },
    "pickup crate": {
        1: {
            "standing": (
                "/subject-1/pickup_crate-49",
                0,
                2,
            ),
            "moving": (
                "/subject-1-moving/pickup_crate-50",
                0,
                2,
            ),
        },
        2: {
            "standing": (
                "/subject-2/pickup_crate-25",
                0,
                2,
            ),
            "moving": (
                "/subject-2-moving/pickup_crate-23",
                0,
                2,
            ),
        },
        3: {
            "standing": (
                "/subject-3/pickup_crate-27",
                0,
                2,
            ),
            "moving": (
                "/subject-3-moving/pickup_crate-36",
                0,
                2,
            ),
        },
        4: {
            "standing": (
                "/subject-4/pickup_crate-32",
                0,
                2,
            ),
            "moving": (
                "/subject-4-moving/pickup_crate-38",
                0,
                2,
            ),
        },
        5: {
            "standing": (
                "/subject-5/pickup_crate-26",
                0,
                2,
            ),
            "moving": (
                "/subject-5-moving/pickup_crate-127",
                0,
                2,
            ),
        },
        6: {
            "standing": (
                "/subject-6/pickup_crate-34",
                0,
                2,
            ),
            "moving": (
                "/subject-6-moving/pickup_crate-29",
                0,
                2,
            ),
        },
        7: {
            "standing": (
                "/subject-7/pickup_crate-24",
                0,
                2,
            ),
            "moving": (
                "/subject-7-moving/pickup_crate-63",
                0,
                2,
            ),
        },
        8: {
            "standing": (
                "/subject-8/pickup_crate-26",
                0,
                2,
            ),
            "moving": (
                "/subject-8-moving/pickup_crate-36",
                0,
                2,
            ),
        },
        9: {
            "standing": (
                "/subject-9/pickup_crate-48",
                0,
                2,
            ),
            "moving": (
                "/subject-9-moving/pickup_crate-66",
                0,
                2,
            ),
        },
        10: {
            "standing": (
                "/subject-10/pickup_crate-27",
                0,
                2,
            ),
            "moving": (
                "/subject-10-moving/pickup_crate-38",
                0,
                2,
            ),
        },
    },
    "deposit crate": {
        1: {
            "standing": (
                "/subject-1/deposit_crate-31",
                0,
                2,
            ),
            "moving": (
                "/subject-1-moving/deposit_crate-35",
                0,
                2,
            ),
        },
        2: {
            "standing": (
                "/subject-2/deposit_crate-13",
                0,
                2,
            ),
            "moving": (
                "/subject-2-moving/deposit_crate-9",
                0,
                2,
            ),
        },
        3: {
            "standing": (
                "/subject-3/deposit_crate-13",
                0,
                2,
            ),
            "moving": (
                "/subject-3-moving/deposit_crate-22",
                0,
                2,
            ),
        },
        4: {
            "standing": (
                "/subject-4/deposit_crate-14",
                0,
                2,
            ),
            "moving": (
                "/subject-4-moving/deposit_crate-20",
                0,
                2,
            ),
        },
        5: {
            "standing": (
                "/subject-5/deposit_crate-13",
                0,
                2,
            ),
            "moving": (
                "/subject-5-moving/deposit_crate-21",
                0,
                2,
            ),
        },
        6: {
            "standing": (
                "/subject-6/deposit_crate-14",
                0,
                2,
            ),
            "moving": (
                "/subject-6-moving/deposit_crate-15",
                0,
                2,
            ),
        },
        7: {
            "standing": (
                "/subject-7/deposit_crate-15",
                0,
                2,
            ),
            "moving": (
                "/subject-7-moving/deposit_crate-47",
                0,
                2,
            ),
        },
        8: {
            "standing": (
                "/subject-8/deposit_crate-13",
                0,
                2,
            ),
            "moving": (
                "/subject-8-moving/deposit_crate-21",
                0,
                2,
            ),
        },
        9: {
            "standing": (
                "/subject-9/deposit_crate-18",
                0,
                2,
            ),
            "moving": (
                "/subject-9-moving/deposit_crate-29",
                0,
                2,
            ),
        },
        10: {
            "standing": (
                "/subject-10/deposit_crate-14",
                0,
                2,
            ),
            "moving": (
                "/subject-10-moving/deposit_crate-20",
                0,
                2,
            ),
        },
    },
    "get crate": {
        1: {
            "standing": (
                "/subject-1/deliver_crate-23",
                1,
                3,
            ),
            "moving": (
                "/subject-1-moving/deliver_crate-30",
                1,
                3,
            ),
        },
        2: {
            "standing": (
                "/subject-2/deliver_crate-6",
                2,
                3,
            ),
            "moving": (
                "/subject-2-moving/deliver_crate-4",
                1,
                3,
            ),
        },
        3: {
            "standing": (
                "/subject-3/deliver_crate-5",
                1,
                3,
            ),
            "moving": (
                "/subject-3-moving/deliver_crate-16",
                1,
                3,
            ),
        },
        4: {
            "standing": (
                "/subject-4/deliver_crate-8",
                1,
                3,
            ),
            "moving": (
                "/subject-4-moving/deliver_crate-13",
                1,
                3,
            ),
        },
        5: {
            "standing": (
                "/subject-5/deliver_crate-6",
                1,
                3,
            ),
            "moving": (
                "/subject-5-moving/deliver_crate-13",
                1,
                3,
            ),
        },
        6: {
            "standing": (
                "/subject-6/deliver_crate-6",
                1,
                3,
            ),
            "moving": (
                "/subject-6-moving/deliver_crate-6",
                1,
                3,
            ),
        },
        7: {
            "standing": (
                "/subject-7/deliver_crate-7",
                1,
                3,
            ),
            "moving": (
                "/subject-7-moving/deliver_crate-40",
                1,
                3,
            ),
        },
        8: {
            "standing": (
                "/subject-8/deliver_crate-6",
                1,
                3,
            ),
            "moving": (
                "/subject-8-moving/deliver_crate-15",
                1,
                3,
            ),
        },
        9: {
            "standing": (
                "/subject-9/deliver_crate-7",
                1,
                3,
            ),
            "moving": (
                "/subject-9-moving/deliver_crate-19",
                1,
                3,
            ),
        },
        10: {
            "standing": (
                "/subject-10/deliver_crate-6",
                1,
                3,
            ),
            "moving": (
                "/subject-10-moving/deliver_crate-11",
                1,
                3,
            ),
        },
    },
    "return crate": {
        1: {
            "standing": (
                "/subject-1/return_crate-59",
                0,
                3,
            ),
            "moving": (
                "/subject-1-moving/return_crate-56",
                0,
                3,
            ),
        },
        2: {
            "standing": (
                "/subject-2/return_crate-32",
                0,
                3,
            ),
            "moving": (
                "/subject-2-moving/return_crate-31",
                0,
                3,
            ),
        },
        3: {
            "standing": (
                "/subject-3/return_crate-35",
                0,
                3,
            ),
            "moving": (
                "/subject-3-moving/return_crate-47",
                0,
                3,
            ),
        },
        4: {
            "standing": (
                "/subject-4/return_crate-38",
                0,
                3,
            ),
            "moving": (
                "/subject-4-moving/return_crate-51",
                0,
                3,
            ),
        },
        5: {
            "standing": (
                "/subject-5/return_crate-31",
                0,
                3,
            ),
            "moving": (
                "/subject-5-moving/return_crate-140",
                0,
                3,
            ),
        },
        6: {
            "standing": (
                "/subject-6/return_crate-43",
                0,
                3,
            ),
            "moving": (
                "/subject-6-moving/return_crate-38",
                0,
                3,
            ),
        },
        7: {
            "standing": (
                "/subject-7/return_crate-33",
                0,
                3,
            ),
            "moving": (
                "/subject-7-moving/return_crate-72",
                0,
                3,
            ),
        },
        8: {
            "standing": (
                "/subject-8/return_crate-34",
                0,
                3,
            ),
            "moving": (
                "/subject-8-moving/return_crate-50",
                0,
                3,
            ),
        },
        9: {
            "standing": (
                "/subject-9/return_crate-60",
                0,
                3,
            ),
            "moving": (
                "/subject-9-moving/return_crate-73",
                0,
                3,
            ),
        },
        10: {
            "standing": (
                "/subject-10/return_crate-35",
                0,
                3,
            ),
            "moving": (
                "/subject-10-moving/return_crate-45",
                0,
                3,
            ),
        },
    },
}

def get_rosbag_from_file(filename, mode='r'):
    try:
        bag = rosbag.Bag(filename, mode)
        return bag
    except Exception as e:
        print("Failed to get rosbag topics info from file {:} with exception: '{:}'".format(filename, e))
        return None


class Player(Thread):

    def __init__(self, filename, topics, start_time, duration):
        super(Player, self).__init__()
        self.bag = get_rosbag_from_file(filename)
        self.msgs = self.bag.read_messages(topics=topics)
        self.pubs = {}
        self.start_timestamp = start_time + self.bag.get_start_time()
        self.end_timestamp = self.start_timestamp + duration


    def run(self):
        for topic, msg, timestamp in self.msgs:
            # topic = "/human_actions_old"
            timestamp = timestamp.to_sec()
            if topic not in self.pubs:
                self.pubs[topic] = rospy.Publisher(topic, msg.__class__, queue_size=10)
                rospy.sleep(1)
            if (timestamp >= self.start_timestamp) and (timestamp <= self.end_timestamp):
                self.pubs[topic].publish(msg)
                rospy.sleep(SLEEP)
    def close(self):
        self.bag.close()



class Recorder(object):
    def __init__(self, filename, topics, klass):
        super(Recorder, self).__init__()
        self.bag = get_rosbag_from_file(filename, 'w')
        self.subs = []
        self.topic = topics[0]
        self.subs.append(rospy.Subscriber(self.topic, klass, self.callback))
        print("Created recorder for {}".format(filename))

    def close(self):
        for sub in self.subs:
            sub.unregister()
        self.bag.flush()
        self.bag.close()

    def callback(self, msg):
        self.bag.write(self.topic, msg, msg.header.stamp)



if __name__ == '__main__':
    rospy.init_node("play_record")
    rosbag_path = ("/data/out-video", ".bag")
    modes = ["standing", "moving"]
    sids = [1,2,3,4,5,6,7,8,9,10]
    topics = ["/camera/color/image_raw"]
    # sids = ["1"]
    # open database
    for behaviour_label, action in data.items():
        for picker_id, rosbag_data in action.items():
            outpath = os.path.join(rosbag_path[0] + "2", "subject-" + str(picker_id))
            try:
                os.makedirs(outpath)
            except OSError:
                if not os.path.isdir(outpath):
                    raise
            outpath = os.path.join(rosbag_path[0] + "2", "subject-" + str(picker_id)+"-moving")
            try:
                os.makedirs(outpath)
            except OSError:
                if not os.path.isdir(outpath):
                    raise
            for mode in modes:
                parameters = rosbag_data[mode]
                subpath = parameters[0]
                start = parameters[1]
                duration = parameters[2]
                source_filename = rosbag_path[0] + subpath + rosbag_path[1]
                target_filename = rosbag_path[0] + "2" + subpath + rosbag_path[1]

                print("starting with {}".format(source_filename))
                recorder = Recorder(target_filename, topics, klass=Image)
                ## TODO: readjust
                # player = Player(os.path.join(source_folder,"{:}".format(filename)), topics=["/human_actions"], start_time=None)
                player = Player(source_filename, topics, start, duration)
                player.start()
                player.join()
                player.close()
                rospy.sleep(1)
                recorder.close()
