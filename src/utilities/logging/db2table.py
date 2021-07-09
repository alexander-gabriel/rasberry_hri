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
WAITING_THRESHOLD = 5

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

    def get_runs(self, experiment_id=None, picker=None):
        with closing(self.db.cursor()) as cursor:
            if picker is not None:
                cursor.execute(("SELECT DISTINCT run_id FROM runs "
                                "WHERE (experiment_id = ? AND picker_id = ?)"),
                               (experiment_id, picker))
            elif experiment_id is not None:
                cursor.execute(("SELECT DISTINCT run_id FROM runs "
                                "WHERE (experiment_id = ?)"),
                               (experiment_id,))
            else:
                cursor.execute("SELECT DISTINCT run_id FROM runs")
            return list(map(lambda x: x[0], cursor.fetchall()))

    def get_goal_choice(self, experiment):
        goal_choice = dotdict()
        goal_choice.succeeded = []
        goal_choice.failed = []
        goal_choice.errors = []
        run_stats = {}
        with closing(self.db.cursor()) as cursor:
            if experiment.picker:
                cursor.execute(("SELECT r.run_id, g.goal FROM robot_goals g, runs r WHERE "
                                "r.experiment_id = ? AND r.run_id = g.run_id AND r.picker_id = ? ORDER BY g.timestamp"),  (experiment.id, experiment.picker))
            else:
                cursor.execute(("SELECT r.run_id, g.goal FROM robot_goals g, runs r WHERE "
                                "r.experiment_id = ? AND r.run_id = g.run_id ORDER BY g.timestamp"),  (experiment.id,))
            for result in cursor.fetchall():
                try:
                    run_stats[result[0]].append(result[1])
                except KeyError:
                    run_stats[result[0]] = [result[1]]
        for run_id, goals in run_stats.items():
            goals = list(filter(lambda x: x != "WaitGoal", goals))
            if experiment.p.expected_goal in goals and (experiment.p.expected_goal == goals[0] or "ApproachGoal" == goals[0]):
                goal_choice.succeeded.append(run_id)
                goal_choice.errors += list(filter(lambda x: x != experiment.p.expected_goal and x != "WaitGoal", goals))
            else:
                goal_choice.failed.append(run_id)
                goal_choice.errors += list(filter(lambda x: x != "WaitGoal", goals))
        succeeded = len(goal_choice.succeeded)
        try:
            goal_choice.success = float(succeeded) / len(run_stats)
        except ZeroDivisionError:
            goal_choice.success = 0
        return goal_choice

    def get_success(self, experiment):

        if experiment.p.picker_behavior in ["deliver no call", "deliver expect mind read", "deliver standard", "deliver expect robot to come", "deliver expect robot to come simple"]:
            behaviour1 = 'get crate'
        elif experiment.p.picker_behavior in ["exchange expect robot to come", "exchange standard", "exchange no call", "exchange expect mind read", "exchange expect robot to come simple"]:
            behaviour1 = 'return crate'
        else:
            behaviour1 = "NOT DEFINED"
            print("no behaviour1: {}".format(experiment.p.picker_behavior))
        if experiment.p.picker_behavior.startswith("deliver"):
            if experiment.p.picker_behavior.endswith("simple"):
                action = "SimpleGiveCrateAction"
            else:
                action = 'GiveCrateAction'
        elif experiment.p.picker_behavior.startswith("exchange"):
            if experiment.p.picker_behavior.endswith("simple"):
                action = 'SimpleExchangeCrateAction'
            else:
                action = 'ExchangeCrateAction'
        else:
            print("no action")
            action = "NOT DEFINED"
        if experiment.p.picker_behavior in ["deliver standard", "deliver no call", "exchange standard", "exchange no call"]:
            location = 13.5
            time_limit = 30
            if experiment.p.robot_x == 5.53:
                location = 11.5

        elif experiment.p.picker_behavior in ["deliver expect robot to come", "deliver expect mind read", "exchange expect robot to come", "exchange expect mind read"]:
            location = 16.2
            time_limit = 30
        elif experiment.p.picker_behavior in ["deliver expect robot to come simple", "exchange expect robot to come simple"]:
            location = 16.2
            time_limit = 30
        else:
            location = 0
            time_limit = 0
            print("no location")
        task_performance = dotdict()
        task_performance.succeeded = []
        task_performance.failed = []
        with closing(self.db.cursor()) as cursor:
            if experiment.picker:
                cursor.execute(("""SELECT p.run_id, p.x - a.end_x - 0.95 as distance, p.timestamp < a.timestamp+6 and p.timestamp > a.timestamp as concurrent, g.duration, a.end_x FROM picker_behavior p, robot_actions a,
     robot_goals g, runs r WHERE  r.experiment_id = ? AND r.picker_id = ? AND r.run_id = p.run_id AND r.run_id = a.run_id AND r.run_id = g.run_id AND g.goal = ? AND p.behavior = ? AND a.action = ?"""),  (experiment.id, experiment.picker, experiment.p.expected_goal, behaviour1, action))
            else:
                cursor.execute(("""SELECT p.run_id, p.x - a.end_x - 0.95 as distance, p.timestamp < a.timestamp+6 and p.timestamp > a.timestamp as concurrent, g.duration, a.end_x FROM picker_behavior p, robot_actions a,
     robot_goals g, runs r WHERE r.run_id = p.run_id AND r.run_id = a.run_id AND r.run_id = g.run_id AND g.goal = ? AND p.behavior = ? AND a.action = ? AND r.experiment_id = ?"""),  (experiment.p.expected_goal, behaviour1, action, experiment.id))
            count = 0
            vergleich = experiment.goal_choice.succeeded[:]
            for result in cursor.fetchall():
                count += 1
                if result[0] in experiment.goal_choice.succeeded:
                    try:
                        vergleich.remove(result[0])
                    except ValueError:
                        pass
                    if float(result[1]) < 0.6 and float(result[1]) > -0.05 and bool(result[2]) and float(result[3]< time_limit):
                        task_performance.succeeded.append(result[0])
                    else:
                        task_performance.failed.append(result[0])
                else:
                    print("unknown run")
            if count == 0:
                task_performance.failed = experiment.goal_choice.succeeded
            task_performance.failed += vergleich
        succeeded = len(task_performance.succeeded)
        print("{} results: {:d}  goal_choice success: {:d}   task performance: {:d} / {:d}".format(experiment.id, count, len(experiment.goal_choice.succeeded), len(task_performance.succeeded), len(task_performance.failed)))
        try:
            task_performance.success = float(succeeded) / len(experiment.goal_choice.succeeded)
        except ZeroDivisionError:
            task_performance.success = 0
        return task_performance

    def get_waiting_times(self, experiment):
        waiting_times = []
        if experiment.p.picker_behavior in ["deliver standard", "deliver expect robot to come", "exchange expect robot to come", "call", "exchange standard"]:
            behaviour1 = 'call robot'
        elif experiment.p.picker_behavior in ["deliver no call", "deliver expect mind read", "exchange no call", "exchange expect mind read"]:
            behaviour1 = 'expect service'
        elif experiment.p.picker_behavior in ["deliver expect robot to come simple", "exchange expect robot to come simple"]:
            behaviour1 = "call robot simple"
        else:
            behaviour1 = "NOT DEFINED"
        with closing(self.db.cursor()) as cursor:
            if experiment.picker:
                cursor.execute(("SELECT r.run_id, g.timestamp - b.timestamp as waiting FROM picker_behavior b, robot_goals g, runs r WHERE "
                                "r.experiment_id = ? AND r.picker_id = ? AND r.run_id = g.run_id AND r.run_id = b.run_id AND b.behavior = ? AND g.goal = ?"), (experiment.id, experiment.picker, behaviour1, experiment.p.expected_goal))
            else:
                cursor.execute(("SELECT r.run_id, g.timestamp - b.timestamp as waiting FROM picker_behavior b, robot_goals g, runs r WHERE "
                                "r.experiment_id = ? AND r.run_id = g.run_id AND r.run_id = b.run_id AND b.behavior = ? AND g.goal = ?"), (experiment.id, behaviour1, experiment.p.expected_goal))
            for result in cursor.fetchall():
                if result[0] in experiment.task_performance.succeeded:
                    if isinstance(result[1], float) and result[1] < WAITING_THRESHOLD:
                        waiting_times.append(float(result[1]))
        return waiting_times

    def get_service_times(self, experiment):

        service_times = []
        if experiment.p.picker_behavior in ["deliver standard", "deliver expect robot to come", "exchange expect robot to come", "call", "exchange standard"]:
            behaviour1 = 'call robot'
        elif experiment.p.picker_behavior in ["deliver no call", "deliver expect mind read", "exchange no call", "exchange expect mind read"]:
            behaviour1 = 'expect service'
        elif experiment.p.picker_behavior in ["deliver expect robot to come simple", "exchange expect robot to come simple"]:
            behaviour1 = "call robot simple"
        else:
            behaviour1 = "NOT DEFINED"
        # if experiment.p.picker_behavior in ["deliver standard", "deliver no call", "exchange standard", "exchange no call"]:
        #     behaviour2 = 'leave with crate'
        # elif experiment.p.picker_behavior in ["deliver expect robot to come", "deliver expect mind read", "exchange expect robot to come", "exchange expect mind read", "deliver expect robot to come simple", "exchange expect robot to come simple"]:
        #     behaviour2 = 'pick berries'
        # else:
        #     behaviour2 = "NOT DEFINED"
        behaviour2 = 'pick berries'
        with closing(self.db.cursor()) as cursor:
            if experiment.picker:
                cursor.execute(("SELECT r.run_id, b2.timestamp - b1.timestamp as duration FROM picker_behavior b1, picker_behavior b2, runs r WHERE "
                                "r.experiment_id = ? AND r.picker_id = ? AND r.run_id = b1.run_id AND r.run_id = b2.run_id AND b1.behavior = ? AND b2.behavior = ? AND b1.timestamp < b2.timestamp"),  (experiment.id, experiment.picker, behaviour1, behaviour2))
            else:
                cursor.execute(("SELECT r.run_id, b2.timestamp - b1.timestamp as duration FROM picker_behavior b1, picker_behavior b2, runs r WHERE "
                                "r.experiment_id = ? AND r.run_id = b1.run_id AND r.run_id = b2.run_id AND b1.behavior = ? AND b2.behavior = ? AND b1.timestamp < b2.timestamp"),  (experiment.id, behaviour1, behaviour2))
            for result in cursor.fetchall():
                if result[0] in experiment.task_performance.succeeded:
                    service_times.append(result[1])
                    if result[1] > 20:
                        print(result[0] + ": " + str(result[1]))
        return service_times

    def get_yaml(self, experiment_id, config_directory=CONFIG_DIRECTORY):
        with closing(self.db.cursor()) as cursor:
            cursor.execute(("SELECT run_id FROM runs WHERE "
                            "experiment_id = ?"),  (experiment_id,))
            run_id = cursor.fetchone()[0]
            with open(os.path.join(config_directory, STATE_DIRECTORY, run_id + '.param'), 'r') as file:
                parameters = yaml.full_load(file)
                return parameters

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
                    if isinstance(result, float):
                        waits.append(float(result))
                        success.append(1.0)
                    else:
                        success.append(0.0)
                        self.failed_runs.append(run_id)
        return success, waits

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

    def get_service2(self, runs, expected_goal):
        success = []
        duration = []
        actual_followed_goals = set()
        failed_runs = []
        for run_id in runs:
            found_expected_goal = False
            with closing(self.db.cursor()) as cursor:
                cursor.execute(("SELECT g.goal, b2.timestamp - b1.timestamp as duration FROM picker_behavior b1, picker_behavior b2, robot_goals g WHERE "
                                "b1.run_id = ? and b2.run_id = b1.run_id and g.run_id = b1.run_id and b1.behaviour = 'expect service' and b2.behaviour = 'leave with crate'"),  (run_id,))
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

    def get_service3(self, runs, expected_goal, experiment_id, behaviour):
        success = []
        if behaviour in ["deliver standard", "deliver expect robot to come", "exchange expect robot to come", "call", "exchange standard"]:
            behaviour1 = 'call robot'
        elif behaviour in ["deliver no call", "deliver expect mind read", "exchange no call", "exchange expect mind read"]:
            behaviour1 = 'expect service'
        else:
            behaviour1 = "NOT DEFINED"
        if behaviour in ["deliver standard", "deliver no call", "exchange standard", "exchange no call"]:
            behaviour2 = 'leave with crate'
        elif behaviour in ["deliver expect robot to come", "deliver expect mind read", "exchange expect robot to come", "exchange expect mind read"]:
            behaviour2 = 'pick berries'
        else:
            behaviour2 = "NOT DEFINED"
        duration = []
        actual_followed_goals = set()
        failed_runs = []
        succeeded_runs = []
        with closing(self.db.cursor()) as cursor:
            cursor.execute(("SELECT b1.run_id, g.goal, b2.timestamp - b1.timestamp as duration FROM picker_behavior b1, picker_behavior b2, robot_goals g, runs e WHERE "
                            "e.run_id = g.run_id AND b2.run_id = b1.run_id and g.run_id = b1.run_id and b1.behaviour = ? and b2.behaviour = ? and e.experiment_id = ? and b1.timestamp < b2.timestamp"),  (behaviour1, behaviour2, experiment_id))
            for result in cursor.fetchall():
                if result[0] in runs:
                    if isinstance(result[2], float) and result[1] == expected_goal:
                        if not result[0] in succeeded_runs:
                            succeeded_runs.append(result[0])
                            duration.append(result[2])
                            # if float(result[1]) < 5:
                            #     print(run_id)
                            success.append(1.0)
                        else:
                            actual_followed_goals.add(result[1])
                    else:
                        success.append(0.0)
                        failed_runs.append(result[0])
                        self.failed_runs.append(result[0])
        return success, duration, actual_followed_goals, failed_runs

    def get_success2(self, runs, expected_goal, experiment_id, behaviour):
        if behaviour in ["deliver no call", "deliver expect mind read", "deliver standard", "deliver expect robot to come"]:
            behaviour1 = 'get crate'
        elif behaviour in ["exchange expect robot to come", "exchange standard", "exchange no call", "exchange expect mind read"]:
            behaviour1 = 'return crate'
        else:
            behaviour1 = "NOT DEFINED"
            print("no behaviour1")
        if behaviour.startswith("deliver"):
            action = 'GiveCrateAction'
        elif behaviour.startswith("exchange"):
            action = 'ExchangeCrateAction'
        else:
            print("no action")
            action = "NOT DEFINED"
        if behaviour in ["deliver standard", "deliver no call", "exchange standard", "exchange no call"]:
            location = 13.5
            time_limit = 11
        elif behaviour in ["deliver expect robot to come", "deliver expect mind read", "exchange expect robot to come", "exchange expect mind read"]:
            location = 16.2
            time_limit = 13
        else:
            location = 0
            time_limit = 0
            print("no location")
        success = []
        successes = []
        fails = []
        picker_x = 0
        picker_y = 0
        with closing(self.db.cursor()) as cursor:
            cursor.execute(("""SELECT p.run_id, p.x - r.end_x - 0.95 as distance, p.timestamp < r.timestamp+6 and p.timestamp > r.timestamp as concurrent, g.duration, r.end_x FROM picker_behavior p, robot_actions r,
 robot_goals g, runs e WHERE e.run_id = p.run_id AND p.run_id = r.run_id AND r.run_id = g.run_id AND g.goal = ? AND p.behaviour = ? AND r.action = ? AND e.experiment_id = ?"""),  (expected_goal, behaviour1, action, experiment_id))
            for result in cursor.fetchall():
                if result[0] in runs:
                    runs.remove(result[0])
                    if float(result[1]) < 1 and float(result[1]) > -0.05 and bool(result[2]) and float(result[3]< time_limit) and abs(float(result[4])-location) < 1:
                        success.append(1.0)
                        successes.append(result[0])
                    else:
                        success.append(0.0)
                        fails.append(result[0])
                else:
                    print("unknown run")
        for run_id in runs:
            success.append(0.0)
        return success, successes, fails

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


class ExperimentResult(object):
    def __init__(self, result):
        self.noise = [result.p.position_noise[0], result.p.posture_noise]
        self.task_performance = result.task_performance.success
        self.goal_choice_success = result.goal_choice.success
        self.waiting = self.calculate_statistics(result.waiting_time)
        duration_without_waiting = list(map(lambda x: x[0] - x[1], zip(result.service_time, result.waiting_time)))
        self.duration = self.calculate_statistics(duration_without_waiting)
        self.total = self.calculate_statistics(result.service_time)
        # self.duration = self.calculate_statistics(result.service_time)
        self.label = result.label
        self.p = result.p
        self.s = result.s
        self.result = result

    def calculate_statistics(self, list):
        return mean(list), var(list)

    def __str__(self):
        output = ""


    def __cmp__(self, other):
        return cmp(self.noise, other.noise)


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class Evaluator(object):
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

    def __init__(self, args):
        self.latex_output = args.l
        self.per_picker = args.p
        self.data = []
        self.behaviour = {}
        self.repeat_these_runs = []
        if args.config:
            self.db = DB(os.path.join(args.config, LOG_DIRECTORY, "log.db"))
        else:
            args.config = CONFIG_DIRECTORY
            self.db = DB()
        try:
            self.delete_state()
        except:
            pass
        if args.id:
            self.experiments = [args.id]
        else:
            self.experiments = self.db.get_experiments()

    def delete_state(self):
        run_ids = self.db.delete_experiment("NO ID SET")
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

    def get_all_results(self):
        speeds = []
        self.all_outputs = {}
        all_results = {}
        for experiment_id in self.experiments:
            parameters, state = self.get_parameters(experiment_id)
            if self.per_picker:
                results = []
                for subject_id in ["picker01", "picker02", "picker03", "picker04", "picker05",
                                   "picker06", "picker07", "picker08", "picker09", "picker10"]:
                    results.append(self.get_experiment_results(experiment_id, parameters, state, subject_id))
                all_results[experiment_id] = results
            else:
                all_results[experiment_id] = [self.get_experiment_results(experiment_id, parameters, state)]
        return all_results

    def get_parameters(self, experiment_id):
        p = dotdict()
        s = dotdict()
        try:
            parameters = self.db.get_yaml(experiment_id, args.config)
            p.picker_behavior = parameters["behaviour"]
            p.gesture = parameters["gesture_perception"]
            p.behavior = parameters["behavior_perception"]
            p.direction = parameters["direction_perception"]
            p.berry = parameters["berry_position_perception"]
            p.robot_crate = parameters["robot_crate_possession_perception"]
            p.picker_crate_possession = parameters["picker_crate_possession_perception"]
            p.picker_crate_fill = parameters["picker_crate_fill_perception"]
            p.robot_x = parameters["robot_position"][0]
            s.berry = not bool(parameters["no_berry_places"])
            s.has_crate = parameters["has_crate"]
            s.crate_full = parameters["crate_full"]
            try:
                p.position_noise = parameters["movement_noise"]
            except:
                p.position_noise = [0, 0]
            try:
                p.posture_noise = parameters["posture_noise"]
            except:
                p.posture_noise = 0
            p.expected_goal = self.get_expected_goal(parameters["experiment_label"])
        except IOError:
            print("No run id for experiment: {}".format(experiment_id))
        return p, s

    def get_experiment_results(self, experiment_id, parameters, state, subject_id=None):

        # label = self.db.get_label(experiment_id)
        # try:
        #     output = self.all_outputs[label]
        # except:
        #     output = []
        #     self.all_outputs[label] = output
        # if self.per_picker:
        #     runs = self.db.get_runs(experiment_id, picker)
        # else:
        #     runs = self.db.get_runs(experiment_id)
        # if not runs:
        #     raise IndexError()

        experiment = dotdict()
        experiment.label = self.db.get_label(experiment_id)
        print(experiment.label)
        print(parameters.posture_noise)
        experiment.id = experiment_id
        experiment.p = parameters
        experiment.s = state
        experiment.picker = subject_id
        # get goal_choice_success
        experiment.goal_choice = self.db.get_goal_choice(experiment)
        # get task_performance
        experiment.task_performance = self.db.get_success(experiment)
        # get waiting time
        experiment.waiting_time = self.db.get_waiting_times(experiment)
        # get service time
        experiment.service_time = self.db.get_service_times(experiment)
        return ExperimentResult(experiment)

    def print_results(self, all_results):
        output = []
        if self.latex_output:
            exp3_results_by_label = {}
            exp4_results_by_label = {}
            for results in all_results.values():
                for result in results:
                    if result.label.startswith("Exp3"):
                        try:
                            exp3_results_by_label[result.label].append(result)
                        except KeyError:
                            exp3_results_by_label[result.label] = [result]
                    elif result.label.startswith("Exp4"):
                        try:
                            exp4_results_by_label[result.label].append(result)
                        except KeyError:
                            exp4_results_by_label[result.label] = [result]
            if exp4_results_by_label:
                output.append("\\begin{tabular}{@{\extracolsep{0pt}} l r@{}l r@{}l r@{}l @{}}")
                output.append("& \\multicolumn{2}{c}{Service [$s$]} & \\multicolumn{2}{c}{Waiting [$s$]} & \\multicolumn{2}{c}{Total [$s$]}\\\\")
                output.append("\\cline{2-7} \\\\")
                output_deliver = "& \\multicolumn{6}{c}{Delivery} \\\\"
                output_deliver_hri_normal = "\\multicolumn{1}{r|}{HRI $6m$} "
                output_deliver_hri_long = "\\multicolumn{1}{r|}{HRI $12m$} "
                output_deliver_hri_comp = "\\multicolumn{1}{r|}{HRI cmp} "
                output_deliver_simple = "\\multicolumn{1}{r|}{Baseline} "
                output_exchange = "& \\multicolumn{6}{c}{Exchange} \\\\"
                output_exchange_hri_normal = "\\multicolumn{1}{r|}{HRI $6m$} "
                output_exchange_hri_long = "\\multicolumn{1}{r|}{HRI $12m$} "
                output_exchange_hri_comp = "\\multicolumn{1}{r|}{HRI cmp} "
                output_exchange_simple = "\\multicolumn{1}{r|}{Baseline} "
                for label, results in exp4_results_by_label.items():
                    parts = label.split(" - ")
                    result = results[0]
                    if parts[1] == "Deliver":
                        if parts[2] == "Normal":
                            output_deliver_hri_normal += "& ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$\\\\".format(result.duration[0], sqrt(result.duration[1]), result.waiting[0], sqrt(result.waiting[1]), result.total[0], sqrt(result.total[1]))
                        elif parts[2] == "Comparable":
                            output_deliver_hri_comp += "& ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$\\\\".format(result.duration[0], sqrt(result.duration[1]), result.waiting[0], sqrt(result.waiting[1]), result.total[0], sqrt(result.total[1]))
                        elif parts[2] == "Simple":
                            output_deliver_simple += "& ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$\\\\".format(result.duration[0], sqrt(result.duration[1]), result.waiting[0], sqrt(result.waiting[1]), result.total[0], sqrt(result.total[1]))
                        elif parts[2] == "Long":
                            output_deliver_hri_long += "& ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$\\\\".format(result.duration[0], sqrt(result.duration[1]), result.waiting[0], sqrt(result.waiting[1]), result.total[0], sqrt(result.total[1]))
                    elif parts[1] == "Exchange":
                        if parts[2] == "Normal":
                            output_exchange_hri_normal += "& ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$\\\\".format(result.duration[0], sqrt(result.duration[1]), result.waiting[0], sqrt(result.waiting[1]), result.total[0], sqrt(result.total[1]))
                        elif parts[2] == "Comparable":
                            output_exchange_hri_comp += "& ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$\\\\".format(result.duration[0], sqrt(result.duration[1]), result.waiting[0], sqrt(result.waiting[1]), result.total[0], sqrt(result.total[1]))
                        elif parts[2] == "Simple":
                            output_exchange_simple += "& ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$\\\\".format(result.duration[0], sqrt(result.duration[1]), result.waiting[0], sqrt(result.waiting[1]), result.total[0], sqrt(result.total[1]))
                        elif parts[2] == "Long":
                            output_exchange_hri_long += "& ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$  & ${:.2f}$&$\pm{:.2f}$\\\\".format(result.duration[0], sqrt(result.duration[1]), result.waiting[0], sqrt(result.waiting[1]), result.total[0], sqrt(result.total[1]))
                output.append(output_deliver)
                output.append(output_deliver_simple)
                output.append(output_deliver_hri_comp)
                output.append(output_deliver_hri_long)
                output.append(output_deliver_hri_normal)


                output.append(output_exchange)
                output.append(output_exchange_simple)
                output.append(output_exchange_hri_comp)
                output.append(output_exchange_hri_long)
                output.append(output_exchange_hri_normal)
                output.append("\\end{tabular}")
            if exp3_results_by_label:
                for label, results in exp3_results_by_label.items():
                    output_tabular = "\\begin{tabular}{l"
                    output_choice_success = "\\multicolumn{1}{r|}{Action Choice [$\\%$]}"
                    output_task_success = "\\multicolumn{1}{r|}{Action Execution [$\\%$]}"
                    output_success = "\\multicolumn{1}{r|}{Success [$\\%$]}"
                    output_duration = "\\multicolumn{1}{r|}{Service Duration [$s$]}"
                    output_waiting = "\\multicolumn{1}{r|}{Waiting Duration [$s$]}"
                    output_total = "\\multicolumn{1}{r|}{Total Duration [$s$]}"
                    output_noise = "\\multicolumn{1}{l}{}"
                    gesture_noise = False
                    cline_count = 1
                    if "gesture" in label:
                        noise_type = "posture"
                        gesture_noise = True
                    else:
                        noise_type = "position"
                    results = sorted(results)
                    for result in results:
                        if (result.noise[1] in [0.3, 0.45, 0.6, 0.75, 0.9]
                              or result.noise[0] in [0.01, 0.015, 0.02, 0.025, 0.03]
                              or (result.noise[0] == 0 and result.noise[1] == 0)):
                            cline_count += 2
                            output_tabular += " r@{}l"
                            if gesture_noise:
                                output_noise += " & \\multicolumn{2}{c}{" + "{:}".format(result.noise[1]) + "}"
                            else:
                                output_noise += " & \\multicolumn{2}{c}{" + "{:}".format(result.noise[0]) + "}"
                            output_choice_success += " & \\multicolumn{2}{c}{" + "{:d}".format(int(result.goal_choice_success*100)) + "}"
                            output_task_success += " & \\multicolumn{2}{c}{" + "{:d}".format(int(result.task_performance*100)) + "}"
                            output_success += " & \\multicolumn{2}{c}{" + "{:d}".format(int(result.goal_choice_success*result.task_performance*100)) + "}"
                            output_waiting += " & ${:.2f}$&$\\pm{:.2f}$".format(result.waiting[0], sqrt(result.waiting[1]))
                            output_total += " & ${:.2f}$&$\\pm{:.2f}$".format(result.total[0], sqrt(result.total[1]))
                            output_duration += " & ${:.2f}$&$\\pm{:.2f}$".format(result.duration[0], sqrt(result.duration[1]))
                            print(result.result.task_performance.failed)
                            # print(result.result.goal_choice.failed)
                    output.append(output_tabular + " }")
                    if gesture_noise:
                        output.append("\\multicolumn{1}{l}{} & \\multicolumn{" + "{:d}".format(cline_count-1) + "}{c}{Posture Noise [$\\%$]} \\\\")
                    else:
                        output.append("\\multicolumn{1}{l}{} & \\multicolumn{" + "{:d}".format(cline_count-1) + "}{c}{Position Noise Variance} \\\\")
                    output.append(output_noise+ "\\\\")
                    output.append("\\cline{2-" + "{:d}".format(cline_count) +  "} \\\\")
                    if "Deliver" in label:
                        output.append("\\multicolumn{1}{l}{} & \\multicolumn{" + "{:d}".format(cline_count-1) + "}{c}{Delivery in case of " + noise_type + " noise}\\\\")
                    else:
                        output.append("\\multicolumn{1}{l}{} & \\multicolumn{" + "{:d}".format(cline_count-1) + "}{c}{Exchange in case of " + noise_type + " noise}\\\\")
                    output.append(output_choice_success + "\\\\")
                    output.append(output_task_success + "\\\\")
                    output.append(output_success + "\\\\")
                    output.append(output_waiting + "\\\\")
                    output.append(output_duration + "\\\\")
                    output.append(output_total + "\\\\")
                    output.append("\end{tabular}")
        else:
            if self.per_picker:
                output.append("label; subject; gesture; behaviour; direction; berry; berry state; robot_crate; picker_crate; has_crate; picker_crate_fill; picker crate full; position noise; posture noise; success; behaviour; duration mean; duration var; wait mean; wait var;exp id")
                for experiment_id, results in all_results.items():
                    for result in results:
                        # runs = self.db.get_runs(experiment_id, subject_id)
                        # if not runs:
                        #     raise IndexError()
                        # # with errstate(all='ignore', divide='ignore'):
                        # success1, duration, actual_followed_goals, failed_runs = self.db.get_service(runs, self.expected_goal)
                        # if len(failed_runs) not in [0, len(runs)]:
                        #     repeat_these_runs += failed_runs
                        # succeeded_runs = list(filter(lambda x: x not in failed_runs, runs))
                        # success2, signal_distances, stop_distances, speed = self.db.get_meetings(runs)
                        output.append("{}; {}; {}; {}; {}; {}; {}; {}; {}; {}; {}"
                            .format(result.label, result.result.picker,
                                    "{}; {}; {}; {}; {}; {}; {}; {}; {}; {}".format(result.p.gesture,
                                                                                result.p.behavior,
                                                                                result.p.direction,
                                                                                result.p.berry,
                                                                                result.s.berry,
                                                                                result.p.robot_crate,
                                                                                result.p.picker_crate_possession,
                                                                                result.s.has_crate,
                                                                                result.p.picker_crate_fill,
                                                                                result.s.crate_full),
                                    "{: >4.3f}; {: >4.3f}".format(result.noise[0], result.noise[1]),
                                    "{:.2}, {:.2}".format(result.goal_choice_success, result.task_performance),
                                    result.p.picker_behavior,
                                    "{: >5.2f}; {: >6.3f}".format(*result.duration),
                                    # "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(signal_distances)), # TODO
                                    # "{:0>5.2f}; {:0>6.3f}".format(*db.calculate_statistics(stop_distances)), # TODO
                                    "{: >5.2f}; {: >6.3f}".format(*result.waiting),
                                    experiment_id,
                                    ", ".join(result.result.goal_choice.failed+result.result.task_performance.failed),
                                    ", ".join(set(result.result.goal_choice.errors))))
            else:
                output.append("label; gesture; behaviour; direction; berry; berry state; robot_crate; picker_crate; has_crate; picker_crate_fill; picker crate full; position noise; posture noise; success; behaviour; duration mean; duration var; wait mean; wait var;exp id")
                for experiment_id, results in all_results.items():
                    result = results[0]
                    # task_success, success_runs, failed_runs = self.db.get_success(runs, self.expected_goal, experiment_id, self.behaviour[experiment_id])
                    # # if self.position_noise == [0.04, 0.04]:
                    # #     print(success_runs)
                    # success1, duration, actual_followed_goals, failed_runs = self.db.get_service3(success_runs, self.expected_goal, experiment_id, self.behaviour[experiment_id])
                    # if len(failed_runs) not in [0, 10]:
                    #     repeat_these_runs+= failed_runs
                    # success2, signal_distances, stop_distances, speed = self.db.get_meetings(success_runs)
                    try:
                        output.append("{}; {}; {}; {}; {}; {}; {}; {}; {}"
                            .format(result.label,
                                "{}; {}; {}; {}; {}; {}; {}; {}; {}; {}".format(result.p.gesture,
                                                                            result.p.behavior,
                                                                            result.p.direction,
                                                                            result.p.berry,
                                                                            result.s.berry,
                                                                            result.p.robot_crate,
                                                                            result.p.picker_crate_possession,
                                                                            result.s.has_crate,
                                                                            result.p.picker_crate_fill,
                                                                            result.s.crate_full),
                                "{: >4.3f}; {: >4.3f}".format(result.noise[0], result.noise[1]),
                                "{:.2f}, {:.2f}".format(result.goal_choice_success, result.task_performance),
                                result.p.picker_behavior,
                                "{: >5.2f}; {: >6.3f}".format(*result.duration),
                                # "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(signal_distances)), # TODO
                                # "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(stop_distances)), # TODO
                                "{: >5.2f}; {: >6.3f}".format(*result.waiting),
                                experiment_id,
                                ", ".join(set(result.result.goal_choice.errors))))
                    except IndexError:
                        output.append("{}; {}; {}; {};  {};  {};  {};  {};  {}; {}; {}; {}"
                            .format(result.label,
                                "{}; {}; {}; {}; {}; {}; {}; {}; {}; {}".format(result.p.gesture,
                                                                            result.p.behavior,
                                                                            result.p.direction,
                                                                            result.p.berry,
                                                                            result.s.berry,
                                                                            result.p.robot_crate,
                                                                            result.p.picker_crate_possession,
                                                                            result.s.has_crate,
                                                                            result.p.picker_crate_fill,
                                                                            result.s.crate_full),
                                "{: >4.3f}; {: >4.3f}".format(result.noise[0], result.noise[1]),
                                "{:.2f}, {:.2f}".format(result.goal_choice_success, result.task_performance),
                                result.p.behaviour,
                                "{: >5.2f}; {: >6.3f}".format(*result.duration),
                                # "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(signal_distances)), # TODO
                                # "{: >5.2f}; {: >6.3f}".format(*db.calculate_statistics(stop_distances)), # TODO
                                "{: >5.2f}; {: >6.3f}".format(*result.waiting),
                                experiment_id))
        for line in output:
            print(line)
        # repeat_these_runs = list(set(repeat_these_runs))
        # print("Failed runs: {}".format(repeat_these_runs))
        # delete_confirm = raw_input('Would you like to delete these runs (y/n)?\n')
        # if delete_confirm == "y":
        #     delete_state(repeat_these_runs)
        #     for run_id in repeat_these_runs:
        #         self.db.delete_run(run_id)
        # print("Speed mean: {} var: {}".format(*db.calculate_statistics(speeds)))

    def get_expected_goal(self, label):
        if label.endswith("Simple"):
            return self.simple_goal_table[label.split(" - ")[1]]
        else:
            return self.goal_table[label.split(" - ")[1]]

if __name__ == '__main__':
    # CONFIG_DIRECTORY = "/home/rasberry"
    # STATE_DIRECTORY="large-state"
    # self.db = DB("/home/rasberry/stop-test-log.db")
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--id', action='store', type=str, help='experiment to show')
    parser.add_argument('-p', action="store_true", help='evaluate per picker')
    parser.add_argument('-l', action="store_true", help='latex compatible output')
    parser.add_argument('--config', type=str, nargs='?', const=CONFIG_DIRECTORY, help='optional config directory')
    args = parser.parse_args()

    evaluator = Evaluator(args)
    results = evaluator.get_all_results()
    evaluator.print_results(results)
