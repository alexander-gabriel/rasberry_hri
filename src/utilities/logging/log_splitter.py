#!/usr/bin/env python2
import re
import os
import argparse
from datetime import datetime
# import numpy as np
import sqlite3
from contextlib import closing

from common.parameters import CONFIG_DIRECTORY, LOG_DIRECTORY, STATE_DIRECTORY

# [rosout][INFO] 2020-03-06 13:38:02,165: SCH: Perceived human action Picker02, calling

call_pattern = re.compile("^\[rosout\]\[INFO\] [0-9]{4,4}-[0-9]{2,2}-[0-9]{2,2} (?P<time>[0-9]{2,2}:[0-9]{2,2}:[0-9]{2,2}),[0-9]{3,3}: SCH: Perceived human action Picker02, calling$")

# [INFO] [1583497616.451514, 97017.075000]: GOL: Performed action <Action:Exchange crate with Picker02>; Action queue is: [<Action:Move to WayPoint103>]

action_done_pattern = re.compile("^\[rosout\]\[INFO\] [0-9]{4,4}-[0-9]{2,2}-[0-9]{2,2} (?P<time>[0-9]{2,2}:[0-9]{2,2}:[0-9]{2,2}),[0-9]{3,3}: GOL: Performed action \<Action:(?P<action>Exchange|Give|Evade|BerryEvade) .*$")


# [INFO] [1583521661.122962, 114113.265000]: ACT: Performing <Action

action_start_pattern = re.compile("^\[rosout\]\[INFO\] [0-9]{4,4}-[0-9]{2,2}-[0-9]{2,2} (?P<time>[0-9]{2,2}:[0-9]{2,2}:[0-9]{2,2}),[0-9]{3,3}: ACT: Performing \<Action:(?P<action>Exchange|Give|Evade|BerryEvade) .*$")

# [INFO] [1583497625.984403, 97023.865000]: BDI: Finished following <Goal:Exchange crate with Picker02 at WayPoint105 (240, 12.375)>

goal_pattern =  re.compile("^\[rosout\]\[INFO\] [0-9]{4,4}-[0-9]{2,2}-[0-9]{2,2} (?P<time>[0-9]{2,2}:[0-9]{2,2}:[0-9]{2,2}),[0-9]{3,3}: BDI: Finished following \<Goal:(?P<goal>Exchange|Deliver|Evade|BerryEvade) .*$")


start_pattern = re.compile("^\[rosout\]\[INFO\] [0-9]{4,4}-[0-9]{2,2}-[0-9]{2,2} (?P<time>[0-9]{2,2}:[0-9]{2,2}:[0-9]{2,2}),[0-9]{3,3}: EXP: Initializing Experiment: (?P<scenario>\w+) - Subject (?P<subject_id>[0-9]+) - Mode (?P<mode>\w+)$")

# [rosout][WARNING] 2020-03-06 02:09:49,297: BDI: Robot has met picker. Distance: 0.471746484403

meeting_pattern = re.compile("^\[rosout\]\[WARNING\] [0-9]{4,4}-[0-9]{2,2}-[0-9]{2,2} (?P<time>[0-9]{2,2}:[0-9]{2,2}:[0-9]{2,2}),[0-9]{3,3}: BDI: Robot has met picker. Distance: (?P<distance>[0-9].[0-9]+)$")


timeout_pattern = re.compile("^\[rosout\]\[INFO\] [0-9]{4,4}-[0-9]{2,2}-[0-9]{2,2} (?P<time>[0-9]{2,2}:[0-9]{2,2}:[0-9]{2,2}),[0-9]{3,3}: EXP: Timeout condition reached$")


# [INFO] [1583521053.848928, 113684.240000]: WST: Picker02 is at WayPoint104
waypoint_pattern = re.compile("^\[rosout\]\[INFO\] [0-9]{4,4}-[0-9]{2,2}-[0-9]{2,2} (?P<time>[0-9]{2,2}:[0-9]{2,2}:[0-9]{2,2}),[0-9]{3,3}: WST: Picker02 is at WayPoint104$")


patterns = ["INFO", "WARNING", "ERROR"]


class DB:
    def __init__(self, filename=os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY, 'log.db')):
        self.db = sqlite3.connect(filename, check_same_thread=False)
        self.failed_runs = []

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

    def close(self):
        self.db.close()


def read_file(lines, filename):
    with open(filename, 'r') as file:
        for line in file.readlines():
            for pattern in patterns:
                if pattern in line:
                    lines.append(line)
                    break


def get_run_data(foldername):
    lines = []
    read_file(lines, os.path.join(foldername, "experiment_node.log"))
    for filename in os.listdir(os.path.join(foldername, "39a9ec4a-847a-11ea-b1ee-00012e83d65b")):
        if filename.endswith(".log"):
            read_file(lines, os.path.join(foldername, "39a9ec4a-847a-11ea-b1ee-00012e83d65b", filename))
    # lines = sorted(lines, key= lambda line: datetime.strptime(line.split(" ")[2].split(",")[0], "%H:%M:%S"))
    lines = sorted(lines, key= lambda line: line.split(" ")[2].split(",")[0])
    experiments = []
    current_experiment = None
    for line in lines:
        match = re.match(start_pattern, line)
        if not match is None:
            group = match.groupdict()
            current_experiment = {}
            current_experiment["start time"] = datetime.strptime(group["time"], "%H:%M:%S")
            current_experiment["subject_id"] = group["subject_id"]
            current_experiment["mode"] = group["mode"]
            current_experiment["scenario"] = group["scenario"]

        match = re.match(waypoint_pattern, line)
        if not match is None:
            group = match.groupdict()
            current_experiment["waypoint time"] = datetime.strptime(group["time"], "%H:%M:%S")

        match = re.match(call_pattern, line)
        if not match is None and not "call time" in current_experiment:
            group = match.groupdict()
            current_experiment["call time"] = datetime.strptime(group["time"], "%H:%M:%S")

        match = re.match(meeting_pattern, line)
        if not match is None and not "meeting time" in current_experiment:
            group = match.groupdict()
            current_experiment["meeting time"] = datetime.strptime(group["time"], "%H:%M:%S")
            try:
                current_experiment["time to meeting"] = current_experiment["meeting time"] - group["call time"]
            except:
                current_experiment["time to meeting"] = current_experiment["meeting time"] - current_experiment["start time"]
            current_experiment["meeting distance"] = group["distance"]

        # match = re.match(action_start_pattern, line)
        # if not match is None:
        #     group = match.groupdict()
        #     current_experiment["service time"] = datetime.strptime(group["time"], "%H:%M:%S")
        #     if group["action"] == "Evade":
        #         current_experiment["time to service"] = current_experiment["service time"] - current_experiment["waypoint time"]


        match = re.match(action_done_pattern, line)
        if not match is None and not "service time" in current_experiment:
            group = match.groupdict()
            current_experiment["service time"] = datetime.strptime(group["time"], "%H:%M:%S")
            if group["action"] == "Evade" or group["action"] == "BerryEvade" :
                current_experiment["time to service"] = current_experiment["service time"] - current_experiment["waypoint time"]
            else:
                try:
                    current_experiment["time to service"] = current_experiment["service time"] - current_experiment["call time"]
                except:
                    current_experiment["time to service"] = current_experiment["service time"] - current_experiment["start time"]

        match = re.match(goal_pattern, line)
        if not match is None and not "end time" in current_experiment:
            group = match.groupdict()
            current_experiment["end time"] = datetime.strptime(group["time"], "%H:%M:%S")
            current_experiment["success"] = True

        match = re.match(timeout_pattern, line)
        if not match is None:
            group = match.groupdict()
            if not "success" in current_experiment:
                current_experiment["end time"] = datetime.strptime(group["time"], "%H:%M:%S")
                current_experiment["time to end"] = current_experiment["end time"] - current_experiment["start time"]
                current_experiment["success"] = False
            experiments.append(current_experiment)
    return experiments


def get_statistics(data_series):
    if not data_series:
        return "\\multicolumn{2}{c|}{N/A}"
    print(data_series)
    return "{:.2f} & {:.3f}".format(np.mean(data_series), np.var(data_series))


def get_success(successes):
    return "{:.2f}".format(float(sum(successes))/len(successes))


class Line(object):
    def __init__(self, line):
        self.line = line
        self.date = datetime.strptime(line.split("] ", 1)[1].split(": ", 1)[0], "%Y-%m-%d %H:%M:%S,%f")
        self.content = line.split(": ", 1)[1]

    def __eq__(self, other):
        """Override the default Equals behavior"""
        if inspect.isclass(other):
            return other.__name__ == self.__class__.__name__
        if isinstance(other, self.__class__):
            return self.content == other.content
        return False

    def __ne__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            # for subgoal in self.subgoal_templates:
            #     if not subgoal in other.subgoal_templates:
            #         return True
            # for subgoal in other.subgoal_templates:
            #     if not subgoal in self.subgoal_templates:
            #         return True
            return not (self.content == other.content)
        return True

    def __gt__(self, other):
        return self.date > other.date

    def __lt__(self, other):
        return self.date < other.date

    def __ge__(self, other):
        return (self > other) or (self == other)

    def __le__(self, other):
        return (self < other) or (self == other)

    def __repr__(self):
        return "{:%Y-%m-%d %H:%M:%S,%f}: {}".format(self.date, self.content)


class Traceback(Line):
    def __init__(self, traceback, last_line):
        first_line = traceback.split("\n", 1)[0]
        try:
            super(Traceback, self).__init__(first_line)
            self.content = traceback.split(": ", 1)[1]
        except:
            super(Traceback, self).__init__(last_line)
            self.line = first_line
            self.content = traceback


    def __repr__(self):
        return "{:%Y-%m-%d %H:%M:%S,%f}: {}".format(self.date, self.content)


def get_logs(path, logs=[]):
    for root, subdirs, files in os.walk(LOG_PATH):
        for filename in files:
            if (filename.startswith("thorvald_001-experimenter_node")
                or filename.startswith("thorvald_001-scheduler")
                or filename.startswith("roslaunch-simulator")
                or filename.startswith("thorvald_001-picker_mover")):
                with open(os.path.join(root, filename), 'r') as file:
                    traceback_mode = False
                    traceback = ""
                    last_line = ""
                    for line in file.readlines():
                        if line.endswith("Traceback (most recent call last):\n"):
                            traceback_mode = True
                        if traceback_mode:
                            if line == "\n":
                                logs.append(Traceback(traceback, last_line))
                                traceback_mode = False
                                traceback = ""
                            else:
                                traceback += line
                        else:
                            try:
                                if line.startswith("[rosout]"):
                                    logs.append(Line(line))
                                    last_line = line
                            except:
                                pass
    return sorted(logs)

def distribute_logs(run_ids):
    for root, subdirs, files in os.walk(os.path.join(CONFIG_DIRECTORY, LOG_DIRECTORY)):
        for filename in files:
            run_id = filename[:-4]
            if run_id in run_ids:
                os.rename(os.path.join(root, filename), os.path.join(args.config, LOG_DIRECTORY, filename))

def check_logs(run_ids):
    missing = []
    other = []
    for root, subdirs, files in os.walk(os.path.join(args.config, LOG_DIRECTORY)):
        for filename in files:
            run_id = filename[:-4]
            try:
                run_ids.remove(run_id)
            except:
                other.append(run_id)
    return run_ids, other


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Split log files per experiment run')
    parser.add_argument('--config', type=str, nargs='?', const=CONFIG_DIRECTORY, help='optional config directory')
    parser.add_argument('--move', action="store_true")
    parser.add_argument('--check', action="store_true")
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
    if args.check:
        run_ids = db.get_runs()
        missing, other = check_logs(run_ids)
        print("missing:")
        print(missing)
        print("other:")
        print(other)
    elif args.move:
        run_ids = db.get_runs()
        distribute_logs(run_ids)
    else:
        LOG_PATH = "/home/rasberry/.ros/log/"
        logs = get_logs(LOG_PATH)
        experiments = {}
        latest_run = None
        during_run = False
        for line in logs:
            if line.content.startswith("EXPE: Starting Experiment run"):
                during_run = True
                run_id = line.content.split("run ", 1)[1][:-1]
                latest_run = run_id
                try:
                    run_data = experiments[run_id]
                except:
                    run_data = {}
                if not "start" in run_data or run_data["start"] < line.date:
                    run_data["start"] = line.date
                    run_data["log"] = [str(line)]
                experiments[run_id] = run_data
            elif line.content.startswith("EXPE: Ending Experiment run"):
                during_run = False
                run_id = line.content.split("run ", 1)[1][:-1]
                run_data = experiments[run_id]
                if not "end" in run_data or run_data["end"] < line.date:
                    run_data["end"] = line.date
                    run_data["log"].append(str(line))
                experiments[run_id] = run_data
            elif during_run:
                experiments[run_id]["log"].append(str(line))

        for run_id, run_data in experiments.items():
            # db.add_log(run_id, run_data)
            # print(run_id + " : " + str(len(run_data["log"])))
            with open(os.path.join(args.config, LOG_DIRECTORY, run_id + ".log"), 'w') as file:
                for line in run_data["log"]:
                    file.write(line)



    # with open(filename, 'r') as file:
    #     for line in file.readlines():

    # search run borders in log files
    # save run-specific logs in individual log files
    #


    #
    #
    # PER_SUBJECT = False
    # experiments = []
    # for experiment in range(1,2+1):
    #     for run in range(1,10+1):
    #         try:
    #             experiments += get_run_data(os.path.join("/", "home", "rasberry", ".ros", "experiment{:d}".format(experiment), "session-{:d}".format(run)))
    #         except:
    #             pass
    # data = {}
    # if PER_SUBJECT:
    #     for e in experiments:
    #         try:
    #             data[e["scenario"]][e["subject_id"]]["success"].append(e["success"])
    #             if e["success"]:
    #                 data[e["scenario"]][e["subject_id"]]["durations"].append(e["time to service"].total_seconds())
    #             try:
    #                 if e["scenario"] != "Evasion" and e["scenario"] != "BerryEvasion":
    #                     data[e["scenario"]][e["subject_id"]]["distances"].append(float(e["meeting distance"]))
    #             except:
    #                 pass
    #         except:
    #             if e["scenario"] not in data:
    #                 data[e["scenario"]] = {}
    #             if e["subject_id"] not in data[e["scenario"]]:
    #                 data[e["scenario"]][e["subject_id"]] = { "success": [], "distances": [], "durations": []}
    #             try:
    #                 if e["scenario"] != "Evasion" and e["scenario"] != "BerryEvasion":
    #                     data[e["scenario"]][e["subject_id"]]["distances"].append(float(e["meeting distance"]))
    #             except:
    #                 pass
    #             data[e["scenario"]][e["subject_id"]]["success"].append(e["success"])
    #             data[e["scenario"]][e["subject_id"]]["durations"].append(e["time to service"].total_seconds())
    #     results = []
    #     for scenario_id, scenario in data.items():
    #         for subject_id, subject in scenario.items():
    #             results.append({"scenario": scenario_id, "subject_id": subject_id, "success": get_success(subject["success"]), "meeting distance": get_statistics(subject["distances"]), "time to service": get_statistics(subject["durations"])})
    #     print("scenario;mode;subject;success;min distance; duration")
    #     for e in results:
    #         if e["success"]:
    #             try:
    #                 print(" & ".join([e["scenario"],e["subject_id"],str(e["success"]),e["meeting distance"],e["time to service"]])+"\\\\")
    #             except:
    #                 print(" & ".join([e["scenario"],e["subject_id"],str(e["success"]),"\\multicolumn{2}{c|}{N/A}",str(e["time to service"])])+"\\\\")
    #         else:
    #             try:
    #                 print(" & ".join([e["scenario"],e["subject_id"],str(e["success"]),"{:.2f}".format(float(e["meeting distance"])),str(e["time to end"])])+"\\\\")
    #             except:
    #                 print(" & ".join([e["scenario"],e["subject_id"],str(e["success"]),"\\multicolumn{2}{c|}{N/A}",str(e["time to end"])])+"\\\\")
    # else:
    #     for e in experiments:
    #         try:
    #             data[e["scenario"]]["success"].append(e["success"])
    #             if e["success"]:
    #                 data[e["scenario"]]["durations"].append(e["time to service"].total_seconds())
    #             try:
    #                 if e["scenario"] != "Evasion" and e["scenario"] != "BerryEvasion":
    #                     data[e["scenario"]]["distances"].append(float(e["meeting distance"]))
    #             except:
    #                 pass
    #         except:
    #             if e["scenario"] not in data:
    #                 data[e["scenario"]] = { "success": [], "distances": [], "durations": []}
    #             try:
    #                 if e["scenario"] != "Evasion" and e["scenario"] != "BerryEvasion":
    #                     data[e["scenario"]]["distances"].append(float(e["meeting distance"]))
    #             except:
    #                 pass
    #             data[e["scenario"]]["success"].append(e["success"])
    #             data[e["scenario"]]["durations"].append(e["time to service"].total_seconds())
    #     results = []
    #     for scenario_id, scenario in data.items():
    #         results.append({"scenario": scenario_id, "success": get_success(scenario["success"]), "meeting distance": get_statistics(scenario["distances"]), "time to service": get_statistics(scenario["durations"])})
    #     print("scenario;mode;subject;success;min distance; duration")
    #     for e in results:
    #         if e["success"]:
    #             try:
    #                 print(" & ".join([e["scenario"],str(e["success"]),e["meeting distance"],e["time to service"]])+"\\\\")
    #             except:
    #                 print(" & ".join([e["scenario"],str(e["success"]),"\\multicolumn{2}{c|}{N/A}",str(e["time to service"])])+"\\\\")
    #         else:
    #             try:
    #                 print(" & ".join([e["scenario"],str(e["success"]),"{:.2f}".format(float(e["meeting distance"])),str(e["time to end"])])+"\\\\")
    #             except:
    #                 print(" & ".join([e["scenario"],str(e["success"]),"\\multicolumn{2}{c|}{N/A}",str(e["time to end"])])+"\\\\")

        # try:
        #     print("Scenario: {} Subject: {} Mode: {} Succes: {} Time to service: {}".format(e["scenario"], e["subject_id"], e["mode"], e["success"], e["time to service"]))
        # except:
        #     try:
        #         print("Scenario: {} Subject: {} Mode: {} Succes: {} Time to meet: {}".format(e["scenario"], e["subject_id"], e["mode"], e["success"], e["time to meeting"]))
        #     except:
        #         print("Scenario: {} Subject: {} Mode: {} Succes: {} Time to end: {}".format(e["scenario"], e["subject_id"], e["mode"], e["success"], e["end time"]))
