#!/usr/bin/env python2
import sqlite3
import os
import json
import argparse
import warnings
# from numpy import mean, var, errstate
import numpy as np
import yaml
from contextlib import closing

from common.parameters import CONFIG_DIRECTORY, LOG_DIRECTORY, STATE_DIRECTORY

def mean(l):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        # warnings.filterwarnings('error')
        try:
            return np.nanmean(l)
        except RuntimeWarning:
            return np.NaN

def var(l):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        return np.var(l)

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
        self.failed_runs = []

    def get_experiments(self):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT DISTINCT experiment_id FROM experiments")
            return list(map(lambda x: x[0], cursor.fetchall()))

    def get_label(self, experiment_id):
        with closing(self.db.cursor()) as cursor:
            cursor.execute("SELECT DISTINCT experiment_label FROM experiments "
                           "WHERE (experiment_id = ?)", (experiment_id,))
            return cursor.fetchall()[0][0]

    def get_runs(self, experiment_id, subject_id=None):
        with closing(self.db.cursor()) as cursor:
            if subject_id is not None:
                cursor.execute(("SELECT DISTINCT run_id FROM runs "
                                "WHERE (experiment_id = ? AND picker_id = ?)"),
                               (experiment_id, subject_id))
            else:
                cursor.execute(("SELECT DISTINCT run_id FROM runs "
                                "WHERE (experiment_id = ?)"),
                               (experiment_id,))
            return list(map(lambda x: x[0], cursor.fetchall()))

    def get_meetings(self, runs):
        signal_distances = []
        stop_distances = []
        speeds = []
        success = []
        for run_id in runs:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(("SELECT signal_distance, stop_distance, speed_profile FROM meetings WHERE "
                                "run_id = ?"),
                               (run_id,))
                results = cursor.fetchall()
                for result in results:
                    result_success = False
                    try:
                        result_speeds = map(lambda x: float(x), result[2].split(", "))
                        speeds += list(filter(lambda number: number > 0, result_speeds))
                    except ValueError:
                        result_speeds = []
                    if result[0] is not None:
                        signal_distances.append(result[0])
                        result_success = True
                    if result[1] is not None:
                        stop_distances.append(result[1])
                        if result_success:
                            success.append(1.0)
                    if not result_success:
                        success.append(0.0)
                        self.failed_runs.append(run_id)
                # for index, entry in enumerate(result_speeds):
                #     speeds[index].append(entry)
        # for index in range(4):
        #     speeds[index] = (mean(speeds[index]), var(speeds[index]))
        return success, signal_distances, stop_distances, speeds

    def get_waits(self, runs):
        waits = []
        success = []
        for run_id in runs:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(("SELECT wait FROM picker_waiting WHERE "
                                "run_id = ?"), (run_id,))
                for result in list(map(lambda x: x[0], cursor.fetchall())):
                    if isinstance(result, float):
                        waits.append(float(result))
                        success.append(1.0)
                    else:
                        success.append(0.0)
                        self.failed_runs.append(run_id)
        return success, waits

    def get_yaml(self, run_id, config_directory=CONFIG_DIRECTORY):
        with open(os.path.join(config_directory, STATE_DIRECTORY, run_id + '.param'), 'r') as file:
            parameters = yaml.full_load(file)
            return parameters

    def get_service(self, runs, expected_goal):
        success = []
        duration = []
        failed_runs = []
        for run_id in runs:
            found_expected_goal = False
            with closing(self.db.cursor()) as cursor:
                cursor.execute(("SELECT goal, duration FROM robot_goals WHERE "
                                "run_id = ?"),  (run_id,))
                for result in cursor.fetchall():
                    if isinstance(result[1], float) and result[0] == expected_goal:
                        found_expected_goal = True
                        duration.append(result[1])
                        success.append(1.0)
            if not found_expected_goal:
                success.append(0.0)
                failed_runs.append(run_id)
                self.failed_runs.append(run_id)
        return success, duration, failed_runs

    def calculate_statistics(self, list):
        return mean(list), var(list)

    def close(self):
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

def letter_cmp(a, b):
    for index in range(4):
        if a[index] > b[index]:
            return 1
        elif a[index] < b[index]:
            return -1
    return 0
from functools import cmp_to_key
letter_cmp_key = cmp_to_key(letter_cmp)

goal_table = {
    "Deliver": "DeliverGoal",
    "Evade": "EvadeGoal",
    "Exchange": "ExchangeGoal",
    "Resupply": "DepositGoal",
    "Leave": "StandByGoal",
    "Approach": "ApproachGoal"
}
def get_expected_goal(label):
    return goal_table[label.split(" - ")[1]]

def delete_state(run_ids):
    path = os.path.join(CONFIG_DIRECTORY, STATE_DIRECTORY, "experiments.json")
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
    # CONFIG_DIRECTORY = "/home/rasberry"
    # STATE_DIRECTORY="large-state"
    # db = DB("/home/rasberry/stop-test-log.db")
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('-p', type=bool, nargs='?', const=False, help='evaluate per picker')
    parser.add_argument('--config', type=str, nargs='?', const=CONFIG_DIRECTORY, help='optional config directory')
    args = parser.parse_args()
    if args.config:
        db = DB(os.path.join(args.config, LOG_DIRECTORY, "log.db"))
    else:
        args.config = CONFIG_DIRECTORY
        db = DB()
    try:
        run_ids = db.delete_experiment("NO ID SET")
        delete_state(run_ids)
    except:
        pass
    experiments = db.get_experiments()
    data = []
    behaviour = {}
    repeat_these_runs = []
    if args.p:
        print("label;subject;success;behaviour;duration mean; duration var; signal_distance mean; signal_distance var; stop_distance mean; stop_distance var; wait mean; wait var; speed mean; speed var; gesture; behavior; direction; berry; robot_crate; picker_crate; picker_crate_fill; exp id")
    else:
        print("label;success;behaviour;duration mean; duration var; signal_distance mean; signal_distance var; stop_distance mean; stop_distance var; wait mean; wait var; speed mean; speed var; gesture; behavior; direction; berry; robot_crate; picker_crate; picker_crate_fill; exp id")
    speeds = []
    for experiment_id in experiments:
        label = db.get_label(experiment_id)
        if args.p:
            for subject_id in [
                        "picker01",
                        "picker02",
                        "picker03",
                        "picker04",
                        "picker05",
                        "picker06", "picker07", "picker08", "picker09", "picker10"
                         ]:
                try:
                    runs = db.get_runs(experiment_id, subject_id)
                    if not runs:
                        raise IndexError()
                    # with errstate(all='ignore', divide='ignore'):
                    parameters = {}
                    try:
                        parameters = db.get_yaml(runs[0], args.config)
                        if experiment_id not in behaviour:
                            behaviour[experiment_id] = parameters["behaviour"]
                    except IOError:
                        print("No run id for experiment: {}".format(experiment_id))
                        behaviour[experiment_id] = "No run ID"
                    gesture_perception = parameters["gesture_perception"]
                    behavior_perception = parameters["behavior_perception"]
                    direction_perception = parameters["direction_perception"]
                    berry_perception = parameters["berry_position_perception"]
                    robot_crate_perception = parameters["robot_crate_possession_perception"]
                    picker_crate_possession_perception = parameters["picker_crate_possession_perception"]
                    picker_crate_fill_perception = parameters["picker_crate_fill_perception"]
                    expected_goal = get_expected_goal(parameters["experiment_label"])
                    success1, duration = db.get_service(runs, expected_goal)
                    success2, signal_distances, stop_distances, speed = db.get_meetings(runs)
                    speeds += speed
                    success3, waits = db.get_waits(runs)
                    print("{}; {}; {}; {}; {};  {};  {};  {};  {};  {}; {}"
                        .format(label, subject_id,
                                "{:.2}, {:.2}, {:.2}".format(mean(success1), mean(success2), mean(success3)),
                                behaviour[experiment_id],
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(duration)),
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(signal_distances)),
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(stop_distances)),
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(waits)),
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(speed)),
                                "{}; {}; {}; {}; {}; {}; {}".format(gesture_perception, behavior_perception, direction_perception, berry_perception, robot_crate_perception, picker_crate_possession_perception, picker_crate_fill_perception),
                                experiment_id))
                except IndexError:
                    pass
        else:
            try:
                runs = db.get_runs(experiment_id)
                if not runs:
                    raise IndexError()
                parameters = {}
                try:
                    parameters = db.get_yaml(runs[0], args.config)
                    if experiment_id not in behaviour:
                        behaviour[experiment_id] = parameters["behaviour"]
                except IOError:
                    print("No run id for experiment: {}".format(experiment_id))
                    behaviour[experiment_id] = "No run ID"
                gesture_perception = parameters["gesture_perception"]
                behavior_perception = parameters["behavior_perception"]
                direction_perception = parameters["direction_perception"]
                berry_perception = parameters["berry_position_perception"]
                robot_crate_perception = parameters["robot_crate_possession_perception"]
                picker_crate_possession_perception = parameters["picker_crate_possession_perception"]
                picker_crate_fill_perception = parameters["picker_crate_fill_perception"]
                expected_goal = get_expected_goal(parameters["experiment_label"])
                success1, duration, failed_runs = db.get_service(runs, expected_goal)
                if len(failed_runs) in [1]:
                    repeat_these_runs+= failed_runs
                success2, signal_distances, stop_distances, speed = db.get_meetings(runs)
                speeds += speed
                success3, waits = db.get_waits(runs)
                print("{}; {}; {}; {};  {};  {};  {};  {};  {}; {}"
                    .format(label,
                            "{:.2}, {:.2}, {:.2}".format(mean(success1), mean(success2), mean(success3)),
                            behaviour[experiment_id],
                            "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(duration)),
                            "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(signal_distances)),
                            "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(stop_distances)),
                            "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(waits)),
                            "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(speed)),
                            "{}; {}; {}; {}; {}; {}; {}".format(gesture_perception, behavior_perception, direction_perception, berry_perception, robot_crate_perception, picker_crate_possession_perception, picker_crate_fill_perception),
                            experiment_id))
            except IndexError:
                pass
    repeat_these_runs = list(set(repeat_these_runs))
    print("Failed runs: {}".format(repeat_these_runs))
    delete_confirm = raw_input('Would you like to delete these runs (y/n)?\n')
    if delete_confirm == "y":
        delete_state(repeat_these_runs)
        for run_id in repeat_these_runs:
            db.delete_run(run_id)
    # print("Speed mean: {} var: {}".format(*db.calculate_statistics(speeds)))
                # print("No runs for {}".format(subject_id))

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
