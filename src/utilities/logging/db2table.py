#!/usr/bin/env python2
import sqlite3
import os
import json
import argparse
import warnings
# from numpy import mean, var, errstate
import numpy as np
import yaml
from math import sqrt
from contextlib import closing

from common.parameters import CONFIG_DIRECTORY, LOG_DIRECTORY, STATE_DIRECTORY
WAITING_THRESHOLD = 0.1

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

    def get_runs(self, experiment_id=None, subject_id=None):
        with closing(self.db.cursor()) as cursor:
            if subject_id is not None:
                cursor.execute(("SELECT DISTINCT run_id FROM runs "
                                "WHERE (experiment_id = ? AND picker_id = ?)"),
                               (experiment_id, subject_id))
            elif experiment_id is not None:
                cursor.execute(("SELECT DISTINCT run_id FROM runs "
                                "WHERE (experiment_id = ?)"),
                               (experiment_id,))
            else:
                cursor.execute("SELECT DISTINCT run_id FROM runs")
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
        return success, signal_distances, stop_distances, speeds

    def get_waits(self, runs):
        waits = []
        success = []
        for run_id in runs:
            with closing(self.db.cursor()) as cursor:
                cursor.execute(("SELECT wait FROM picker_waiting WHERE "
                                "run_id = ?"), (run_id,))
                for result in list(map(lambda x: x[0], cursor.fetchall())):
                    if isinstance(result, float) and result < WAITING_THRESHOLD:
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
        actual_followed_goals = set()
        failed_runs = []
        for run_id in runs:
            found_expected_goal = False
            with closing(self.db.cursor()) as cursor:
                cursor.execute(("SELECT goal, duration FROM robot_goals WHERE "
                                "run_id = ?"),  (run_id,))
                for result in cursor.fetchall():
                    if isinstance(result[1], float) and result[0] == expected_goal and not found_expected_goal:
                        found_expected_goal = True
                        duration.append(result[1])
                        success.append(1.0)
                    elif not found_expected_goal:
                        actual_followed_goals.add(result[0])
            if not found_expected_goal:
                success.append(0.0)
                failed_runs.append(run_id)
                self.failed_runs.append(run_id)
        return success, duration, actual_followed_goals, failed_runs

    def get_success(self, runs, expected_goal):
        success = []
        if expected_goal == "DeliverGoal":
            picker_x = 0
            picker_y = 0
            with closing(self.db.cursor()) as cursor:
                cursor.execute(("""SELECT p.run_id, p.x - r.end_x - 0.95 as distance, p.timestamp-5 < r.timestamp+r.duration and p.timestamp > r.timestamp-5 as concurrent, g.duration, r.end_x FROM picker_behavior p, robot_actions r,
	 robot_goals g WHERE p.run_id = r.run_id AND g.run_id = r.run_id AND g.goal = "DeliverGoal" AND p.behavior = ? AND r.action = ?"""),  ("get crate", "GiveCrateAction"))
                for result in cursor.fetchall():
                    if result[0] in runs:
                        runs.remove(result[0])
                        if float(result[1]) < 1 and float(result[1]) > 0.05 and bool(result[2]) and float(result[3]< 12) and abs(float(result[4])-13.5) < 1:
                            success.append(1.0)
                        else:
                            success.append(0.0)
            for run_id in runs:
                success.append(0.0)
            return success
        else:
            print(expected_goal)

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

simple_goal_table = {
    "Deliver": "SimpleDeliverGoal",
    "Exchange": "SimpleExchangeGoal"
}

def get_expected_goal(label):
    if label.endswith("Simple"):
        return simple_goal_table[label.split(" - ")[1]]
    else:
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


class ExperimentResult(object):
    def __init__(self, noise, success, waiting, duration):
        self.noise = noise
        self.success = success
        self.waiting = waiting
        self.duration = duration

    def __cmp__(self, other):
        return cmp(self.noise, other.noise)


if __name__ == '__main__':
    # CONFIG_DIRECTORY = "/home/rasberry"
    # STATE_DIRECTORY="large-state"
    # db = DB("/home/rasberry/stop-test-log.db")
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--id', action='store', type=str, help='experiment to show')
    parser.add_argument('-p', action="store_true", help='evaluate per picker')
    parser.add_argument('-l', action="store_true", help='latex compatible output')
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
    if args.id:
        experiments = [args.id]
    else:
        experiments = db.get_experiments()
    data = []
    behaviour = {}
    repeat_these_runs = []
    if not args.l:
        if args.p:
            print("label;subject;success;behaviour;duration mean; duration var; signal_distance mean; signal_distance var; stop_distance mean; stop_distance var; wait mean; wait var; speed mean; speed var; gesture; behavior; direction; berry; berry state; robot_crate; picker_crate; picker_crate_fill; exp id")
        else:
            print("label;gesture; behavior; direction; berry; berry state; robot_crate; picker_crate; has_crate; picker_crate_fill; picker crate full; position noise; posture noise; success; behaviour; duration mean; duration var; signal_distance mean; signal_distance var; stop_distance mean; stop_distance var; wait mean; wait var; speed mean; speed var;exp id")
    speeds = []
    all_outputs = {}

    for experiment_id in experiments:
        label = db.get_label(experiment_id)
        try:
            output = all_outputs[label]
        except:
            output = []
            all_outputs[label] = output
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
                    berry_existence = not bool(parameters["no_berry_places"])
                    robot_crate_perception = parameters["robot_crate_possession_perception"]
                    picker_crate_possession_perception = parameters["picker_crate_possession_perception"]
                    picker_crate_fill_perception = parameters["picker_crate_fill_perception"]
                    expected_goal = get_expected_goal(parameters["experiment_label"])
                    success1, duration, actual_followed_goals, failed_runs = db.get_service(runs, expected_goal)
                    if len(failed_runs) not in [0, len(runs)]:
                        repeat_these_runs += failed_runs
                    succeeded_runs = list(filter(lambda x: x not in failed_runs, runs))
                    success2, signal_distances, stop_distances, speed = db.get_meetings(runs)
                    speeds += speed
                    success3, waits = db.get_waits(runs)
                    print("{}; {}; {}; {}; {};  {};  {};  {};  {};  {}; {}; {}; {}"
                        .format(label, subject_id,
                                "{:.2}, {:.2}, {:.2}".format(mean(success1), mean(success2), mean(success3)),
                                behaviour[experiment_id],
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(duration)),
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(signal_distances)),
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(stop_distances)),
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(waits)),
                                "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(speed)),
                                "{}; {}; {}; {}; {}; {}; {}; {}".format(gesture_perception, behavior_perception, direction_perception, berry_perception, berry_existence, robot_crate_perception, picker_crate_possession_perception, picker_crate_fill_perception),
                                experiment_id,
                                ", ".join(failed_runs),
                                ", ".join(actual_followed_goals)))
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
                berry_existence = not bool(parameters["no_berry_places"])
                robot_crate_perception = parameters["robot_crate_possession_perception"]
                picker_crate_possession_perception = parameters["picker_crate_possession_perception"]
                picker_crate_fill_perception = parameters["picker_crate_fill_perception"]
                has_crate = parameters["has_crate"]
                crate_full = parameters["crate_full"]
                try:
                    position_noise = parameters["movement_noise"]
                except:
                    position_noise = [0, 0]
                try:
                    posture_noise = parameters["posture_noise"]
                except:
                    posture_noise = 0
                expected_goal = get_expected_goal(parameters["experiment_label"])
                success1, duration, actual_followed_goals, failed_runs = db.get_service(runs, expected_goal)
                if len(failed_runs) not in [0, 10]:
                    repeat_these_runs+= failed_runs
                success2, signal_distances, stop_distances, speed = db.get_meetings(runs)
                speeds += speed
                success3, waits = db.get_waits(runs)
                test_success = db.get_success(runs, expected_goal)
                if args.l:
                    output.append(ExperimentResult([position_noise[0], posture_noise], mean(test_success), db.calculate_statistics(waits), db.calculate_statistics(duration)))
                else:
                    try:
                        output.append("{}; {}; {}; {};  {};  {};  {};  {};  {}; {}; {}; {}"
                            .format(label,
                                    "{}; {}; {}; {}; {}; {}; {}; {}; {}; {}".format(gesture_perception,
                                                                                behavior_perception,
                                                                                direction_perception,
                                                                                berry_perception,
                                                                                berry_existence,
                                                                                robot_crate_perception,
                                                                                picker_crate_possession_perception,
                                                                                has_crate,
                                                                                picker_crate_fill_perception,
                                                                                crate_full),
                                    "{: >4.3f}; {: >4.3f}".format(position_noise[0], posture_noise),
                                    "{:.2}, {:.2}, {:.2}".format(mean(success1), mean(success2), mean(success3)),
                                    behaviour[experiment_id],
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(duration)),
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(signal_distances)),
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(stop_distances)),
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(waits)),
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(speed)),
                                    experiment_id,
                                    ", ".join(actual_followed_goals)))
                    except:
                        output.append("{}; {}; {}; {};  {};  {};  {};  {};  {}; {}; {}; {}"
                            .format(label,
                                    "{}; {}; {}; {}; {}; {}; {}; {}; {}; {}".format(gesture_perception,
                                                                                behavior_perception,
                                                                                direction_perception,
                                                                                berry_perception,
                                                                                berry_existence,
                                                                                robot_crate_perception,
                                                                                picker_crate_possession_perception,
                                                                                has_crate,
                                                                                picker_crate_fill_perception,
                                                                                crate_full),
                                    "{: >4.3f}; {:}".format(position_noise[0], posture_noise),
                                    "{:.2}, {:.2}, {:.2}".format(mean(success1), mean(success2), mean(success3)),
                                    behaviour[experiment_id],
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(duration)),
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(signal_distances)),
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(stop_distances)),
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(waits)),
                                    "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(speed)),
                                    experiment_id,
                                    ", ".join(actual_followed_goals)))
            except IndexError:
                pass
    for label, outputs in all_outputs.items():
        gesture = False
        if "gesture" in label:
            noise = "posture"
            gesture = True
        else:
            noise = "position"
        cline_count = 1
        output_tabular = "\\begin{tabular}{l"
        output_success = "\\multicolumn{1}{r|}{Success [$\\%$]}"
        output_duration = "\\multicolumn{1}{r|}{Duration [$s$]}"
        output_waiting = "\\multicolumn{1}{r|}{Waiting [$s$]}"
        output_noise = "\\multicolumn{1}{l}{}"
        # print(label)
        outputs = sorted(outputs)
        for output in outputs:
            if args.l:
                if   (output.noise[1] in [0.3, 0.5, 0.6, 0.7, 0.8, 0.85]
                      # or output.noise[0] in [0.2, 0.4, 0.6, 0.8]
                      or True
                      or (output.noise[0] == 0 and output.noise[1] == 0)):
                    cline_count += 2
                    output_tabular += " r@{}l"
                    if gesture:
                        output_noise += " & \\multicolumn{2}{c}{" + "{:}".format(output.noise[1]) + "}"
                    else:
                        output_noise += " & \\multicolumn{2}{c}{" + "{:}".format(output.noise[0]) + "}"
                    output_success += " & \\multicolumn{2}{c}{" + "{:d}".format(int(output.success*100)) + "}"
                    output_waiting += " & ${:.2f}$&$\\pm{:.2f}$".format(output.waiting[0], sqrt(output.waiting[1]))
                    output_duration += " & ${:.2f}$&$\\pm{:.2f}$".format(output.duration[0], sqrt(output.duration[1]))
            else:
                print(output)
        if args.l:
            print(output_tabular + " }")
            if "gesture" in label:
                print("\\multicolumn{1}{l}{} & \\multicolumn{" + "{:d}".format(cline_count-1) + "}{c}{Posture Noise [$\\%$]} \\\\")
            else:
                print("\\multicolumn{1}{l}{} & \\multicolumn{" + "{:d}".format(cline_count-1) + "}{c}{Position Noise Variance} \\\\")
            print(output_noise+ "\\\\")
            print("\\cline{2-" + "{:d}".format(cline_count) +  "} \\\\")
            if "Deliver" in label:
                print("\\multicolumn{1}{l}{} & \\multicolumn{" + "{:d}".format(cline_count-1) + "}{c}{Delivery in case of " + noise + " noise}\\\\")
            else:
                print("\\multicolumn{1}{l}{} & \\multicolumn{" + "{:d}".format(cline_count-1) + "}{c}{Exchange in case of " + noise + " noise}\\\\")
            print(output_success + "\\\\")
            print(output_waiting + "\\\\")
            print(output_duration + "\\\\")
        print("\end{tabular}")
    repeat_these_runs = list(set(repeat_these_runs))
    # print("Failed runs: {}".format(repeat_these_runs))
    # delete_confirm = raw_input('Would you like to delete these runs (y/n)?\n')
    # if delete_confirm == "y":
    #     delete_state(repeat_these_runs)
    #     for run_id in repeat_these_runs:
    #         db.delete_run(run_id)
    # print("Speed mean: {} var: {}".format(*db.calculate_statistics(speeds)))
