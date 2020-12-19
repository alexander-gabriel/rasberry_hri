#!/usr/bin/env python2
import sqlite3
import os
from numpy import mean, var

from common.parameters import CONFIG_DIRECTORY, LOG_DIRECTORY



# datenbank datei lesen
#
# aus einem table indexe und daten lesen
#
# entsprechende eintraege aus anderen tables lesen
#
# dictionary pro experiment nach metriken erstellen
#
# tabellen aus dict erstellen und ausgeben


class DB:
    def __init__(self, filename=os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, 'log.db')):
        self.db = sqlite3.connect(filename, check_same_thread=False)

    def get_experiments(self):
        cursor = self.db.cursor()
        cursor.execute("SELECT DISTINCT experiment_id FROM robot_actions")
        return cursor.fetchall()

    def get_runs(self, experiment_id):
        cursor = self.db.cursor()
        cursor.execute(("SELECT DISTINCT run_id FROM robot_actions "
                        "WHERE (experiment_id = ?)"),
                       (experiment_id,))
        return cursor.fetchall()

    def get_results(self, rows):
        for experiment_id, picker_id, run_id in rows:
            cursor.execute(("SELECT bla FROM picker_behavior WHERE"
                            "(experiment_id = ? AND picker_id = ?"
                            "AND run_id = ?)"),
                           (experiment_id, picker_id, run_id))
            cursor.execute(("SELECT bla FROM picker_waiting WHERE"
                            "(experiment_id = ? AND picker_id = ?"
                            "AND run_id = ?)"),
                           (experiment_id, picker_id, run_id))
            cursor.execute(("SELECT bla FROM robot_actions WHERE"
                            "(experiment_id = ? AND picker_id = ?"
                            "AND run_id = ?)"),
                           (experiment_id, picker_id, run_id))
            cursor.execute(("SELECT bla FROM robot_goals WHERE"
                            "(experiment_id = ? AND picker_id = ?"
                            "AND run_id = ?)"),
                           (experiment_id, picker_id, run_id))
            cursor.execute(("SELECT bla FROM meetings WHERE"
                            "(experiment_id = ? AND picker_id = ?"
                            "AND run_id = ?)"),
                           (experiment_id, picker_id, run_id))

    def build_db(self):
        cursor = self.db.cursor()
        # cursor.execute("CREATE TABLE movement_variance (experiment_id text,"
        #                "timestamp float, x float, y float, typ text,"
        #                "target text)")
        cursor.execute("CREATE TABLE experiments (experiment_label text, experiment_id text)")
        cursor.execute("CREATE TABLE picker_behavior (experiment_id text,"
                       "picker_id text, run_id text, timestamp float, x float, y float,"
                       "orientation float, behaviour text)")
        cursor.execute("CREATE TABLE picker_waiting (experiment_id text,"
                       "picker_id text, run_id text, timestamp float, x float, y float,"
                       "orientation float, wait float)")
        # cursor.execute("CREATE TABLE robot_behavior (experiment_id text,"
        #                "timestamp float, x float, y float, orientation float,"
        #                "behaviour text)")
        cursor.execute("CREATE TABLE robot_actions (experiment_id text,"
                       "picker_id text, run_id text, timestamp float, duration float,"
                       "start_x float, start_y float, end_x float, end_y float,"
                       "action text, info text)")
        cursor.execute("CREATE TABLE robot_goals (experiment_id text,"
                       "picker_id text, run_id text, timestamp float, duration float,"
                       "start_x float, start_y float, end_x float, end_y float,"
                       "goal text)")
        cursor.execute("CREATE TABLE meetings (experiment_id text,"
                       "picker_id text, run_id text, timestamp float, x float, y float,"
                       "distance float, speed_profile text)")
        self.db.commit()
        cursor.close()

    def close_db(self):
        self.db.close()

def letter_cmp(a, b):
    for index in range(4):
        if a[index] > b[index]:
            return 1
        elif a[index] < b[index]:
            return -1
    return 0
from functools import cmp_to_key
letter_cmp_key = cmp_to_key(letter_cmp)


def get_meetings(experiment_id, runs):
    signal_distances = []
    stop_distances = []
    speeds = []
    for run_id in map(lambda x : x[0], runs):
        cursor.execute(("SELECT signal_distance, stop_distance, speed_profile FROM meetings WHERE"
                        " experiment_id = ? AND run_id = ?"),
                       (experiment_id, run_id))
        results = cursor.fetchall()
        for result in results:
            try:
                result_speeds = map(lambda x: float(x), result[2].split(", "))
                speeds += result_speeds
            except ValueError:
                result_speeds = []
            if result[0] is not None:
                signal_distances.append(result[0])
            if result[1] is not None:
                stop_distances.append(result[1])
            # for index, entry in enumerate(result_speeds):
            #     speeds[index].append(entry)
    # for index in range(4):
    #     speeds[index] = (mean(speeds[index]), var(speeds[index]))
    return (mean(signal_distances), var(signal_distances)), (mean(stop_distances), var(stop_distances)), (mean(speeds), var(speeds))


def get_waits(experiment_id, runs):
    waits = []
    for run_id in map(lambda x : x[0], runs):
        cursor.execute(("SELECT wait FROM picker_waiting WHERE"
                        " experiment_id = ? AND run_id = ?"),
                       (experiment_id, run_id))
        results = cursor.fetchall()
        for result in results:
            waits.append(float(result[0]))
    return (mean(waits), var(waits))



def get_service(experiment_id, runs):
    success = []
    duration = []
    for run_id in map(lambda x : x[0], runs):
        cursor.execute(("SELECT duration FROM robot_goals WHERE"
                        " experiment_id = ? AND run_id = ?"),
                       (experiment_id, run_id))
        results = cursor.fetchall()
        for result in results:
            if isinstance(result[0], float):
                duration.append(result[0])
                success.append(1.0)
            else:
                success.append(0.0)
    success = mean(success)
    duration = (mean(duration), var(duration))
    return success, duration


if __name__ == '__main__':
    db = DB()
    experiments = db.get_experiments()
    cursor = db.db.cursor()
    data = []
    print("id;success;duration mean; duration var; signal_distance mean; signal_distance var; stop_distance mean; stop_distance var; speed mean; speed var")
    for experiment_id in map(lambda x : x[0], experiments):
        runs = db.get_runs(experiment_id)

        success, duration = get_service(experiment_id, runs)
        signal_distances, stop_distances, speed = get_meetings(experiment_id,runs)
        print("{}; {:.2f};   {:0>5.2f}; {:0>6.3f};   {:0>5.2f}; {:0>6.3f};   {:0>5.2f}; {:0>6.3f};   {:0>5.2f}; {:0>6.3f}".format(experiment_id, success, duration[0], duration[1], signal_distances[0], signal_distances[1], stop_distances[0], stop_distances[1], speed[0], speed[1]))

        # print("{};{:.2f};{:.3f}".format(experiment_id, waits[experiment_id][0], waits[experiment_id][1]))
        # print("{};{:.2f};{:.3f};{:.2f};{:.3f};{:.2f};{:.3f};{:.2f};{:.3f};{:.2f};{:.3f}".format(experiment_id, distances[experiment_id][0], distances[experiment_id][1], speeds[experiment_id][0][0], speeds[experiment_id][0][1], speeds[experiment_id][1][0], speeds[experiment_id][1][1], speeds[experiment_id][2][0], speeds[experiment_id][2][1], speeds[experiment_id][3][0], speeds[experiment_id][3][1]))

        # experiment_id, picker_id, run_id = row
        # cursor.execute(("SELECT timestamp, behaviour FROM picker_behavior WHERE"
        #                 " experiment_id = ? AND picker_id = ? AND run_id = ?"),
        #                (experiment_id, picker_id, run_id))
    #     results = cursor.fetchall()
    #     for result in results:
    #         data.append([experiment_id, picker_id, run_id, result[0], "HUMAN", result[1]])
    #     cursor.execute(("SELECT timestamp, action FROM robot_actions WHERE"
    #                     " experiment_id = ? AND picker_id = ? AND run_id = ? AND NOT action = ?"),
    #                    (experiment_id, picker_id, run_id, "WaitAction"))
    #     results = cursor.fetchall()
    #     for result in results:
    #         data.append([experiment_id, picker_id, run_id, result[0], "ROBOT", result[1]])
    #     cursor.execute(("SELECT timestamp, goal FROM robot_goals WHERE"
    #                     " experiment_id = ? AND picker_id = ? AND run_id = ? AND NOT goal = ?"),
    #                    (experiment_id, picker_id, run_id, "WaitGoal"))
    #     results = cursor.fetchall()
    #     for result in results:
    #         data.append([experiment_id, picker_id, run_id, result[0], "ROBOT", result[1]])
    # data.sort(key=letter_cmp_key)
    #
    # for entry in data:
    #     print("{} {} {} {:.3f} {} {}".format(*entry))
