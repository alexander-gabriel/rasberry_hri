#!/usr/bin/env python2
import sqlite3
import os
import argparse
import warnings
# from numpy import mean, var, errstate
import numpy as np
import yaml
from contextlib import closing

from common.parameters import CONFIG_DIRECTORY, LOG_DIRECTORY, STATE_DIRECTORY


class DB:
    def __init__(self, filename=os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, 'log.db')):
        self.db = sqlite3.connect(filename, check_same_thread=False)

    def build_db(self):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("CREATE TABLE runs (run_id text PRIMARY KEY, experiment_id text, picker_id text)")
            cursor.execute("CREATE TABLE experiments (experiment_id text PRIMARY KEY, experiment_label text, "
                           "FOREIGN KEY (experiment_id) REFERENCES runs (experiment_id))")
            cursor.execute("CREATE TABLE picker_behavior (run_id text, timestamp float, "
                           "x float, y float, orientation float, behavior text, "
                           "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
            cursor.execute("CREATE TABLE picker_waiting (run_id text, timestamp float, "
                           "x float, y float, orientation float, wait float, "
                           "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
            cursor.execute("CREATE TABLE robot_actions (run_id text, timestamp float, "
                           "duration float, start_x float, start_y float, "
                           "end_x float, end_y float, action text, info text, "
                           "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
            cursor.execute("CREATE TABLE robot_goals (run_id text, timestamp float, "
                           "duration float, start_x float, start_y float, "
                           "end_x float, end_y float, goal text, "
                           "FOREIGN KEY (run_id) REFERENCES runs (run_id))")
            cursor.execute("CREATE TABLE meetings (run_id text, timestamp float, "
                           "x float, y float, distance float, speed_profile text, "
                           "FOREIGN KEY (run_id) REFERENCES runs (run_id))")

    def add_run(self, run_id, experiment_id, experiment_label, picker_id):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("INSERT INTO runs VALUES (?, ?, ?)", (run_id, experiment_id, picker_id))
            cursor.execute("INSERT INTO experiments VALUES (?, ?)", (experiment_id, label))

    def add_picker_waiting(self, entry):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("INSERT INTO picker_waiting VALUES (?, ?, ?, ?, ?, ?)", entry)

    def add_picker_behavior(self, entry):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("INSERT INTO picker_behavior VALUES (?, ?, ?, ?, ?, ?)", entry)

    def add_meeting(self, entry):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("INSERT INTO meetings VALUES (?, ?, ?, ?, ?, ?)", entry)

    def add_robot_goal(self, entry):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("INSERT INTO robot_goals VALUES (?, ?, ?, ?, ?, ?, ?, ?)", entry)

    def add_robot_action(self, entry):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("INSERT INTO robot_actions VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)", entry)

    def get_robot_goals(self):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT run_id, timestamp, duration, start_x, start_y, "
                           "end_x, end_y, goal FROM robot_goals")
            return cursor.fetchall()

    def get_robot_actions(self):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT run_id, timestamp, duration, start_x, start_y, "
                           "end_x, end_y, action, info FROM robot_actions")
            return cursor.fetchall()

    def get_picker_behavior(self):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT run_id, timestamp, duration, x, y, orientation, "
                           "behaviour FROM picker_behavior")
            return cursor.fetchall()

    def get_picker_waiting(self):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT run_id, timestamp, duration, x, y, orientation, "
                           "wait FROM picker_waiting")
            return cursor.fetchall()

    def get_meetings(self):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT run_id, timestamp, x, y, distance, speed_profile "
                           "FROM meetings")
            return cursor.fetchall()

    def get_runs(self):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT DISTINCT a.run_id, a.experiment_id, b.experiment_label, a.picker_id "
                           "FROM (SELECT DISTINCT run_id, experiment_id, picker_id FROM picker_behavior "
                                "UNION "
                                "SELECT DISTINCT run_id, experiment_id, picker_id FROM picker_waiting "
                                "UNION "
                                "SELECT DISTINCT run_id, experiment_id, picker_id FROM robot_goals "
                                "UNION "
                                "SELECT DISTINCT run_id, experiment_id, picker_id FROM robot_actions "
                                "UNION "
                                "SELECT DISTINCT run_id, experiment_id, picker_id FROM meetings) a "
                            "INNER JOIN experiments b ON a.experiment_id = b.experiment_id;")
            return cursor.fetchall():

    def close(self):
        self.db.commit()
        self.db.close()



if __name__ == '__main__':
    # CONFIG_DIRECTORY = "/home/rasberry"
    # STATE_DIRECTORY="large-state"
    # db = DB("/home/rasberry/stop-test-log.db")
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('dbpath', metavar='N', type=str, nargs='?',
                        help='optional db path')
    args = parser.parse_args()
    if args.dbpath:
        db = DB(args.dbpath)
        new_db = DB(args.dbpath+"new")
    else:
        db = DB()
        new_db = DB(os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, 'log.dbnew'))
    new_db.build_db()

    for run in db.get_runs():
        new_db.add_run(*run)
    for meeting in db.get_meetings():
        new_db.add_meeting(meeting)
    for behavior in db.get_picker_behavior():
        new_db.add_picker_behavior(behavior)
    for waiting in db.get_picker_waiting():
        new_db.add_picker_waiting(waiting)
    for action in db.get_robot_actions():
        new_db.add_robot_action(action)
    for goal in db.get_robot_goals():
        new_db.add_robot_goal(goal)
    new_db.close()
    db.close()
