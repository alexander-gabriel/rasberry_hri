#!/usr/bin/env python2
import sqlite3
import os
import argparse
from common.parameters import CONFIG_DIRECTORY, LOG_DIRECTORY, STATE_DIRECTORY
# CONFIG_DIRECTORY="/home/MAYANNA/einalex/work/docker/volumes/rasberry-source/_data/hri_config"
# LOG_DIRECTORY="log"

class DB:
    def __init__(self, filename=os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, 'log.db')):
        self.db = sqlite3.connect(filename, check_same_thread=False)

    def close(self):
        self.db.commit()
        self.db.close()

    def delete_experiment(self, experiment_id):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("DELETE FROM picker_behavior WHERE experiment_id = ?", (experiment_id,))
            cursor.execute("DELETE FROM picker_waiting WHERE experiment_id = ?", (experiment_id,))
            cursor.execute("DELETE FROM robot_actions WHERE experiment_id = ?", (experiment_id,))
            cursor.execute("DELETE FROM robot_goals WHERE experiment_id = ?", (experiment_id,))
            cursor.execute("DELETE FROM meetings WHERE experiment_id = ?", (experiment_id,))
            cursor.execute("DELETE FROM experiments WHERE experiment_id = ?", (experiment_id,))
            self.db.commit()

    def get_experiments(self, experiment_label):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT experiment_id FROM experiments WHERE experiment_label = ?", (experiment_label,))
            ids = []
            for result in cursor.fetchall():
                ids.append(result[0])
            return ids

def delete_state(id):
    path = os.path.join(CONFIG_DIRECTORY, STATE_DIRECTORY, id + ".param")
    try:
        os.remove(path)
    except:
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--id', action='store', type=str, help='experiment to delete')
    parser.add_argument('--label', action='store', type=str, help='scenario to delete')
    parser.add_argument('--dbpath', action='store', type=str, help='optional db path')
    args = parser.parse_args()
    if args.dbpath:
        db = DB(args.dbpath)
    else:
        db = DB()
    if args.id:
        db.delete_experiment(args.id)
        delete_state(args.id)
    if args.label:
        ids = db.get_experiments(args.label)
        for id in ids:
            db.delete_experiment(id)
            delete_state(id)

    db.close()
