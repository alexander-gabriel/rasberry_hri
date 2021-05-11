#!/usr/bin/env python2
import sqlite3
import os
import argparse
import json
from contextlib import closing

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
        runs = []
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT run_id FROM runs WHERE experiment_id = ?", (experiment_id,))
            runs = list(map(lambda x: x[0], cursor.fetchall()))
        for run_id in runs:
            self.delete_run(run_id)
        with closing(self.db.cursor()) as cursor:
            cursor.execute("DELETE FROM experiments WHERE experiment_id = ?", (experiment_id,))
        self.db.commit()
        return runs

    def delete_run(self, run_id):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("DELETE FROM picker_behavior WHERE run_id = ?", (run_id,))
            cursor.execute("DELETE FROM picker_waiting WHERE run_id = ?", (run_id,))
            cursor.execute("DELETE FROM robot_actions WHERE run_id = ?", (run_id,))
            cursor.execute("DELETE FROM robot_goals WHERE run_id = ?", (run_id,))
            cursor.execute("DELETE FROM meetings WHERE run_id = ?", (run_id,))
            cursor.execute("DELETE FROM runs WHERE run_id = ?", (run_id,))
            self.db.commit()

    def get_experiments(self, experiment_label):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT experiment_id FROM experiments WHERE experiment_label = ?", (experiment_label,))
            return list(map(lambda x: x[0], cursor.fetchall()))

    def get_runs(self, experiment_id):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT run_id FROM runs WHERE experiment_id = ?", (experiment_id,))
            return list(map(lambda x: x[0], cursor.fetchall()))

    def get_picker_runs(self, picker_id):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT run_id FROM runs WHERE picker_id = ?", (picker_id,))
            return list(map(lambda x: x[0], cursor.fetchall()))

def delete_state(run_ids, config):
    path = os.path.join(config, STATE_DIRECTORY, "experiments.json")
    with open(path, "r") as file:
        state = json.load(file)
    for id in run_ids:
        try:
            state["finished experiments"].remove(id)
        except ValueError:
            pass
        param_path = os.path.join(CONFIG_DIRECTORY, STATE_DIRECTORY, id + ".param")
        try:
            os.remove(param_path)
        except:
            pass
    with open(path, "w") as file:
        json.dump(state, file, indent=4)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--id', action='store', type=str, help='experiment to delete')
    parser.add_argument('--label', action='store', type=str, help='scenario to delete')
    parser.add_argument('--run', action='store', type=str, help='run to delete')
    parser.add_argument('--config', action='store', type=str, help='optional db path')
    parser.add_argument('--picker', action='store', type=str, help='picker id to remove')
    args = parser.parse_args()
    if args.config:
        db = DB(os.path.join(args.config, LOG_DIRECTORY, "log.db"))
    else:
        args.config = CONFIG_DIRECTORY
        db = DB()
    if args.run:
        delete_state([args.run], args.config)
        db.delete_run(args.run)
    if args.id:
        run_ids = db.delete_experiment(args.id)
        delete_state(run_ids, args.config)
    if args.label:
        ids = db.get_experiments(args.label)
        for id in ids:
            run_ids = db.delete_experiment(id)
            delete_state(run_ids, args.config)
    if args.picker:
        run_ids = db.get_picker_runs(args.picker)
        delete_state(run_ids, args.config)
        for run_id in run_ids:
            db.delete_run(run_id)
    db.close()
