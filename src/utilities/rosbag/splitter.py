#!/usr/bin/env python
import os
import sys

import rosbag
from db import DB

PATH = "/data/"
TOPIC = "/camera/color/image_raw"

def get_parts(sid, db):
    cursor = db.db.cursor()
    sid = sid.split("-")
    subject = sid[0]
    if len(sid) > 1:
        robot = "moving"
    else:
        robot = "stationary"
    cursor.execute("SELECT * from parts WHERE subject = {:} AND robot = '{:}'".format(subject, robot))
    return sorted(cursor.fetchall(), key=lambda entry: entry[0])
    # timestamp, type, distance, robot, subject


def convert_bag(sid, db):
    in_path = os.path.join(PATH, "subject-{:}.bag".format(sid))
    out_path = os.path.join(PATH, "subject-{:}".format(sid))
    try:
        os.mkdir(out_path)
    except:
        pass
    parts = get_parts(sid, db)
    with rosbag.Bag(in_path, "r") as in_bag:
        bag_time = in_bag.get_start_time()
        for index in range(len(parts)):
            label = parts[index][1]
            if label:
                msgs = []
                start_time = parts[index][0]
                try:
                    end_time = parts[index+1][0]
                except:
                    end_time = in_bag.get_end_time()
                for topic, msg, t in in_bag.read_messages(topics=[TOPIC]):
                    secs = t.to_sec()
                    if secs >= start_time and secs <= end_time:
                        msgs.append(msg)
                if len(msgs) > 0:
                    msgs = sorted(msgs, key=lambda msg: msg.header.seq)
                    with rosbag.Bag(os.path.join(out_path,"{:}-{:d}.bag".format(label, int(start_time - bag_time))), 'w') as out_bag:
                        for msg in msgs:
                            out_bag.write(TOPIC, msg, msg.header.stamp)



if __name__ == '__main__':
    ids = [2,3,4,5,6,7,8,9,10]
    sids = []
    for id in ids:
        sids.append(str(id))
        sids.append("{}-moving".format(id))
    sids = ["2", "7", "7-moving", "8-moving"]
    db = DB(os.path.join(PATH,"timestamps.db"))
    # open database
    for sid in sids:
        try:
            print("starting with {}".format(sid))
            convert_bag(sid, db)
        except Exception as err:
            print(err)
