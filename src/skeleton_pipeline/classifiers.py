from math import max

from numpy import arctan2, abs, mean

from poses import pose_list


class MinimumDifferenceClassifier:

    def __init__(self):
        self.limit = 20

    def classify(self, angles, positions):
        errors = dict()
        min_error = ('Initial', 1000000)
        for name, pose in pose_list.items():
            error_list = list()
            for label, target in pose.items():
                angle_or_position, direction, limit = target.split(" ")
                if angle_or_position == "a":
                    try:
                        limit = float(limit)
                    except:
                        limit = angles[limit]
                    if direction == "=":
                        error_list.append(abs(limit - angles[label]))
                    elif direction == ">":
                        error_list.append(max(0, limit - angles[label]))
                    elif direction == "<":
                        error_list.append(max(0, angles[label] - limit))
                    else:
                        pass
                else:
                    try:
                        limit = float(limit)
                    except:
                        limit = posistions[limit]
                    if direction == "=":
                        error_list.append(abs(limit - posistions[label]))
                    elif direction == ">":
                        error_list.append(max(0, limit - posistions[label]))
                    elif direction == "<":
                        error_list.append(max(0, posistions[label] - limit))
                    else:
                        pass
            error = mean(error_list)
            errors[name] = error
            if error < min_error[1]:
                min_error = (name, error)
        return min_error
