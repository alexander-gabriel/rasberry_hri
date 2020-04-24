#!/usr/bin/env python
import re
import os
from datetime import datetime
import numpy as np


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



if __name__ == '__main__':
    PER_SUBJECT = False
    experiments = []
    for experiment in range(1,2+1):
        for run in range(1,10+1):
            try:
                experiments += get_run_data(os.path.join("/", "home", "rasberry", ".ros", "experiment{:d}".format(experiment), "session-{:d}".format(run)))
            except:
                pass
    data = {}
    if PER_SUBJECT:
        for e in experiments:
            try:
                data[e["scenario"]][e["subject_id"]]["success"].append(e["success"])
                if e["success"]:
                    data[e["scenario"]][e["subject_id"]]["durations"].append(e["time to service"].total_seconds())
                try:
                    if e["scenario"] != "Evasion" and e["scenario"] != "BerryEvasion":
                        data[e["scenario"]][e["subject_id"]]["distances"].append(float(e["meeting distance"]))
                except:
                    pass
            except:
                if e["scenario"] not in data:
                    data[e["scenario"]] = {}
                if e["subject_id"] not in data[e["scenario"]]:
                    data[e["scenario"]][e["subject_id"]] = { "success": [], "distances": [], "durations": []}
                try:
                    if e["scenario"] != "Evasion" and e["scenario"] != "BerryEvasion":
                        data[e["scenario"]][e["subject_id"]]["distances"].append(float(e["meeting distance"]))
                except:
                    pass
                data[e["scenario"]][e["subject_id"]]["success"].append(e["success"])
                data[e["scenario"]][e["subject_id"]]["durations"].append(e["time to service"].total_seconds())
        results = []
        for scenario_id, scenario in data.items():
            for subject_id, subject in scenario.items():
                results.append({"scenario": scenario_id, "subject_id": subject_id, "success": get_success(subject["success"]), "meeting distance": get_statistics(subject["distances"]), "time to service": get_statistics(subject["durations"])})
        print("scenario;mode;subject;success;min distance; duration")
        for e in results:
            if e["success"]:
                try:
                    print(" & ".join([e["scenario"],e["subject_id"],str(e["success"]),e["meeting distance"],e["time to service"]])+"\\\\")
                except:
                    print(" & ".join([e["scenario"],e["subject_id"],str(e["success"]),"\\multicolumn{2}{c|}{N/A}",str(e["time to service"])])+"\\\\")
            else:
                try:
                    print(" & ".join([e["scenario"],e["subject_id"],str(e["success"]),"{:.2f}".format(float(e["meeting distance"])),str(e["time to end"])])+"\\\\")
                except:
                    print(" & ".join([e["scenario"],e["subject_id"],str(e["success"]),"\\multicolumn{2}{c|}{N/A}",str(e["time to end"])])+"\\\\")
    else:
        for e in experiments:
            try:
                data[e["scenario"]]["success"].append(e["success"])
                if e["success"]:
                    data[e["scenario"]]["durations"].append(e["time to service"].total_seconds())
                try:
                    if e["scenario"] != "Evasion" and e["scenario"] != "BerryEvasion":
                        data[e["scenario"]]["distances"].append(float(e["meeting distance"]))
                except:
                    pass
            except:
                if e["scenario"] not in data:
                    data[e["scenario"]] = { "success": [], "distances": [], "durations": []}
                try:
                    if e["scenario"] != "Evasion" and e["scenario"] != "BerryEvasion":
                        data[e["scenario"]]["distances"].append(float(e["meeting distance"]))
                except:
                    pass
                data[e["scenario"]]["success"].append(e["success"])
                data[e["scenario"]]["durations"].append(e["time to service"].total_seconds())
        results = []
        for scenario_id, scenario in data.items():
            results.append({"scenario": scenario_id, "success": get_success(scenario["success"]), "meeting distance": get_statistics(scenario["distances"]), "time to service": get_statistics(scenario["durations"])})
        print("scenario;mode;subject;success;min distance; duration")
        for e in results:
            if e["success"]:
                try:
                    print(" & ".join([e["scenario"],str(e["success"]),e["meeting distance"],e["time to service"]])+"\\\\")
                except:
                    print(" & ".join([e["scenario"],str(e["success"]),"\\multicolumn{2}{c|}{N/A}",str(e["time to service"])])+"\\\\")
            else:
                try:
                    print(" & ".join([e["scenario"],str(e["success"]),"{:.2f}".format(float(e["meeting distance"])),str(e["time to end"])])+"\\\\")
                except:
                    print(" & ".join([e["scenario"],str(e["success"]),"\\multicolumn{2}{c|}{N/A}",str(e["time to end"])])+"\\\\")

        # try:
        #     print("Scenario: {} Subject: {} Mode: {} Succes: {} Time to service: {}".format(e["scenario"], e["subject_id"], e["mode"], e["success"], e["time to service"]))
        # except:
        #     try:
        #         print("Scenario: {} Subject: {} Mode: {} Succes: {} Time to meet: {}".format(e["scenario"], e["subject_id"], e["mode"], e["success"], e["time to meeting"]))
        #     except:
        #         print("Scenario: {} Subject: {} Mode: {} Succes: {} Time to end: {}".format(e["scenario"], e["subject_id"], e["mode"], e["success"], e["end time"]))
